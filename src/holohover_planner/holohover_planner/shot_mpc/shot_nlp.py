"""
Inner problem P(B) for one fixed bounce sequence (Sections 2 to 5).

Built once with CasADi SX and solved with Fatrop; the sequence only enters
through the parameters y_bar and h_bar (the virtual goal and the half-width of its
mouth), so one solver serves all of B_K.

Decision vector w, in the stage order Fatrop needs (state of stage k, then its controls):
    stage 0:        x_0, t_f, u_0
    stage 0<k<N:    x_k, t_f, u_{k-1}, u_k, s_box[k-1], s_av[k-1]
    stage N:        x_N, t_f, phi, v+, zeta, s_box[N-1]
Fatrop has no global variables and a stage-wise cost, hence t_f copied into every stage's state and
u_{k-1} too, for the rate term. Rows go the same way: dynamics out of stage k, then its path rows.
- X (4 x N+1), U (2 x N): robot states (x, y, vx, vy) and accelerations (a_x, a_y)
  on the grid t_f / N; the heading is left to the tracker
- phi: angle of the contact normal n = (cos phi, sin phi), left unwrapped
- v+: post-impact puck velocity, lifted with bound v+_x >= eps so every
  division by v+_x is safe at every iterate
- zeta = 1 - gamma (x_g - x_c) / v+_x, lifted with bounds [ZETA_MIN, 1]: this is the
  reach constraint gamma (x_g - x_c) < v+_x, and T_B = -ln(zeta) / gamma is always defined
- s_box, s_av: L1 slacks of the soft table/own-half and puck-avoidance constraints

Parameters p = (x0_hat, u_prev, y_bar, h_bar, q0, v0), with (q0, v0) the puck state at the
planning instant. The puck flies freely up to the impact, since t_f <= tau1 (Section 1).

Initial guesses w0 (Section 7): the previous solution shifted to now (warm), or a
closed-form guess from the virtual-goal aim and the inverted impact (cold).
"""

import time
from dataclasses import dataclass

import numpy as np
import casadi as ca

from .model import GAMMA_EPS, discrete_matrices, flight_phi, free_flight, impact, invert, zoh_interval
from .solve_pool import SolvePool

NX, NU = 4, 2  # robot state (x, y, vx, vy) and input (a_x, a_y)
ZETA_MIN = 1e-3  # floor of zeta for the strict reach inequality; T_B <= -ln(ZETA_MIN) / gamma
MAX_INPUT = 100.0  # Fatrop spins forever inside C++, past max_iter, on NaN or absurd w0 and p: they are not solved


@dataclass
class Solution:
    w: np.ndarray    # decision vector, in stage order
    X: np.ndarray    # robot state (x, y, vx, vy)
    U: np.ndarray    # robot input (a_x, a_y)
    tf: float        # final time of the plan
    phi: float       # angle of the contact normal n = (cos phi, sin phi)
    p_c: np.ndarray  # puck centre at impact, exact by the contact equality
    vp: np.ndarray   # post-impact puck velocity
    zeta: float      # reach margin (unitless), lifted with bounds [ZETA_MIN, 1] so T_B = -ln(zeta) / gamma is always defined. zeta = 1 - gamma (x_g - x_c) / v+_x
    cost: float      # cost
    status: str      # Fatrop return status, "failed" if it raised or was not called
    iters: int       # Fatrop iteration count
    t_wall: float    # wall-clock time of the solve
    valid: bool      # True if converged and cost, viol, slack are finite and <= valid_tol
    viol: float      # max violation of the hard constraints
    slack: float     # max violation of the soft constraints
    cold: bool       # True if solved with the cold guess
    terms: dict      # values of the cost terms A-E and of T_B, e_aim, v_n (Section 5)


class ShotNLP:
    def __init__(self, P, verbose=True):
        self.P = P
        self.pool = None  # worker processes of the outer updates (start_pool); None: serial
        N = P.N
        gam = P.gamma

        ###########################
        ### Decision variables  ###
        ###########################
        X = ca.SX.sym("X", NX, N + 1)   # X0, ..., XN
        U = ca.SX.sym("U", NU, N)       # U0, ..., U_{N-1}
        T = ca.SX.sym("T", N + 1)       # copy of tf in each stage
        U_last = ca.SX.sym("U_last", NU, N - 1)  # U0, ..., U_{N-2}, copied into the stage after
        ph = ca.SX.sym("phi")           # angle of the contact normal n = (cos phi, sin phi)
        vp = ca.SX.sym("vp", 2)         # post-impact puck velocity
        zeta = ca.SX.sym("zeta")        # reach margin, lifted with bounds [ZETA_MIN, 1] so T_B = -ln(zeta) / gamma is always defined
        s_box = ca.SX.sym("s_box", N)   # L1 slacks of the soft table/own-half constraints
        s_av = ca.SX.sym("s_av", N - 1) # L1 slacks of the soft puck-avoidance constraints
        tf = T[N]                       # the impact rows live in stage N

        idx = {}  # block name -> indices in w
        blocks, nx, nu = [], [], []
        for k in range(N + 1):
            state = [("X", X[:, k]), ("tf", T[k])] + ([("U_last", U_last[:, k - 1])] if 0 < k < N else [])
            ctrl = [("U", U[:, k])] if k < N else [("phi", ph), ("vp", vp), ("zeta", zeta)]
            ctrl += [("s_box", s_box[k - 1])] if k > 0 else []
            ctrl += [("s_av", s_av[k - 1])] if 0 < k < N else []
            nx.append(sum(b.numel() for _, b in state))
            nu.append(sum(b.numel() for _, b in ctrl))
            blocks += [b for _, b in state + ctrl]
            o = sum(nx) + sum(nu) - nx[-1] - nu[-1]
            for name, b in state + ctrl:
                idx.setdefault(name, []).extend(range(o, o + b.numel()))
                o += b.numel()
        self.idx = {name: np.array(i) for name, i in idx.items()}
        w = ca.vertcat(*blocks)  # decision vector
        self.nw = w.numel()

        ##################
        ### Parameters ###
        ##################
        x0 = ca.SX.sym("x0", NX)
        u_prev = ca.SX.sym("u_prev", NU)
        ybar = ca.SX.sym("ybar")
        hbar = ca.SX.sym("hbar")  # half-width of the goal image's mouth
        q0 = ca.SX.sym("q0", 2) # puck position at the planning instant
        v0 = ca.SX.sym("v0", 2) # puck velocity at the planning instant
        p = ca.vertcat(x0, u_prev, ybar, hbar, q0, v0)
        self.np_ = p.numel() # numel = nummber of elements

        def puck_at(t):
            return free_flight(q0, v0, t, gam)

        ###################
        ### Constraints ###
        ###################
        g, lbg, ubg, ng = [], [], [], []

        def add(expr, lo, hi):
            g.append(expr)
            lbg.extend([lo] * expr.numel())
            ubg.extend([hi] * expr.numel())

        # Impact at t_f (Section 3)
        p_c, v_minus = puck_at(tf)
        n = ca.vertcat(ca.cos(ph), ca.sin(ph))
        v_n, vp_expr, vR_plus = impact(X[2:4, N], v_minus, n, P.kappa, P.m_p / P.m_R, P.mu, P.e) # vp_expr is the post-impact puck velocity expression

        x_c, y_c = p_c[0], p_c[1]
        aim = (y_c - ybar) * vp[0] + (P.x_g - x_c) * vp[1]              # equation of section 4 to make sure we score a goal

        # Soft table limits, whole robot in own half (nodes 1..N)
        x_hi = P.x_mid - P.r_R
        y_hi = 0.5 * P.W - P.r_R
        box_viol = [] # log constraint violation
        p_clear = X[0:2, N] + vR_plus * flight_phi(P.t_clear, gam)   # position of the robot at t_clear after impact
        clear_rows = ca.vertcat(P.r_R - p_clear[0], p_clear[1] - y_hi, -y_hi - p_clear[1]) # condition p_clear[0] >= r_R and |p_clear[1]| <= y_hi
        x_stop = X[0, N] + vR_plus[0] * P.t_react + vR_plus[0] * ca.fabs(vR_plus[0]) / (2 * P.u_max[0]) # furthest x the robot can reach after t_clear if it brakes at max acceleration, so it can stop before the wall

        # Soft puck avoidance before impact (nodes 1..N-1).
        av_viol = [] # log constraint violation
        d_min = P.r_R + P.r_p + P.delta / N

        for k in range(N + 1):
            # Dynamics out of stage k: x_{k+1} = A_d x_k + B_d u_k with the exact ZOH map as expressions of t_f.
            # Fatrop's sign: x_{k+1} - F(x_k, u_k)
            if k < N:
                A, B = discrete_matrices(T[k] / N, gam)
                add(X[:, k + 1] - (A @ X[:, k] + B @ U[:, k]), 0.0, 0.0)
                add(T[k + 1] - T[k], 0.0, 0.0)
                if k < N - 1:
                    add(U_last[:, k] - U[:, k], 0.0, 0.0)

            # Path rows of stage k
            n_rows = len(lbg)
            if k == 0:
                add(X[:, 0] - x0, 0.0, 0.0)
            else:
                rows = ca.vertcat(X[0, k] - x_hi, P.r_R - X[0, k], X[1, k] - y_hi, -y_hi - X[1, k]) # condition X[0, k] in [r_R, x_hi] and X[1, k]| <= y_hi
                if k == N:
                    add(X[0:2, N] - (p_c - (P.r_R + P.r_p) * n), 0.0, 0.0)
                    add(vp - vp_expr, 0.0, 0.0)                                 # contact equality: the puck is sent along n, turned by the friction
                    # Coarse post-impact model (Section 4)
                    add(zeta * vp[0] - (vp[0] - gam * (P.x_g - x_c)), 0.0, 0.0) # define zeta
                    add(v_n, P.v_min, ca.inf)                                   # v_n = (v_R - v_p) . n, robot must move toward the puck
                    add(aim - hbar * vp[0], -ca.inf, 0.0)
                    add(-aim - hbar * vp[0], -ca.inf, 0.0)                      # because of absolute value.
                    rows = ca.vertcat(rows, x_stop - x_hi)
                    if P.t_clear > 0:
                        rows = ca.vertcat(rows, clear_rows)
                add(rows - s_box[k - 1], -ca.inf, 0.0) # soft constraints
                box_viol.append(ca.mmax(rows)) # log the max violation
                if k < N:
                    pp, _ = puck_at(k * T[k] / N)
                    row = (d_min**2 - ca.sumsqr(X[0:2, k] - pp)) / (2 * d_min)
                    add(row - s_av[k - 1], -ca.inf, 0.0)
                    av_viol.append(row)
            ng.append(len(lbg) - n_rows)

        g = ca.vertcat(*g)
        self.lbg = np.array(lbg, dtype=float)
        self.ubg = np.array(ubg, dtype=float)
        equality = (self.lbg == self.ubg).tolist()

        ############
        ### Cost ###
        ############
        # Terms A-E of Section 5, inputs normalised componentwise by their limits
        umax = P.u_max[:NU]
        J_u, J_du = 0, 0
        for k in range(N):
            ub = U[:, k] / umax
            J_u += T[k] * ca.sumsqr(ub)
            J_du += ca.sumsqr(ub - (u_prev if k == 0 else U_last[:, k - 1]) / umax)

        T_B = -ca.log(zeta) / gam if gam >= GAMMA_EPS else (P.x_g - x_c) / vp[0]
        # Aim error on the goal line: the unfolded one, unstretched by e_w^k = h_eff / h_bar
        e_aim = ((y_c - ybar) + (P.x_g - x_c) * vp[1] / vp[0]) * (P.h_eff / hbar)

        terms = {
            "u": P.w_u / N * J_u,
            "du": P.w_du * J_du,
            "tf": P.w_t * tf / P.t_ref,
            "T": P.w_T * T_B / P.T_ref,
            "aim": P.w_a * (e_aim / P.h_eff) ** 2, # more freedom if we remove it
            "slack": P.w_slack * (ca.sum1(s_box) + ca.sum1(s_av)),
        }
        J = sum(terms.values())
        self.term_names = list(terms) + ["T_B", "e_aim", "v_n"]
        self.terms_fun = ca.Function("terms", [w, p], [ca.vertcat(*terms.values(), T_B, e_aim, v_n)])

        # Exposed for tests and for initialising slacks
        self.f_fun = ca.Function("f", [w, p], [J])
        self.g_fun = ca.Function("g", [w, p], [g])
        self.grad_f_fun = ca.Function("grad_f", [w, p], [ca.gradient(J, w)])
        self.jac_g_fun = ca.Function("jac_g", [w, p], [ca.jacobian(g, w)])
        self.viol_fun = ca.Function("viol", [w, p], [ca.vertcat(*box_viol), ca.vertcat(*av_viol)])

        ##############
        ### Bounds ###
        ##############
        self.lbx = -np.inf * np.ones(self.nw)
        self.ubx = np.inf * np.ones(self.nw)
        self.lbx[self.idx["U"]] = np.tile(-umax, N)
        self.ubx[self.idx["U"]] = np.tile(umax, N)
        self.i_tf = self.idx["tf"][0] # the copies follow from the dynamics; define after
        i_vp = self.idx["vp"][0] # vp = speed of the puck after impact
        self.lbx[i_vp] = P.eps
        i_z = self.idx["zeta"][0]
        self.lbx[i_z], self.ubx[i_z] = ZETA_MIN, 1.0
        for name in ("s_box", "s_av"): # slack variable are nonnegative
            self.lbx[self.idx[name]] = 0.0

        ##############
        ### Solver ###
        ##############
        # Interpreted SX, no JIT: compiling the functions to C made a solve only 2-7 % faster
        # (measured with IPOPT and MUMPS, which took the rest of the time) for ~2.4 s of build per run.
        nlp = {"x": w, "p": p, "f": J, "g": g}
        t_build = time.perf_counter()
        self._build_solver(nlp, nx, nu, ng, equality)
        self.t_build = time.perf_counter() - t_build
        if verbose:
            print(f"[shot_nlp] {self.nw} variables, {g.numel()} constraints, "
                  f"build {self.t_build:.2f} s")

    def start_pool(self, n_workers):
        """Solve the bounce sequences of the outer updates in n_workers processes (solve_pool).
        n_workers <= 1 keeps the solves serial."""
        if n_workers > 1:
            self.pool = SolvePool(self, n_workers)
        return self.pool

    def _build_solver(self, nlp, nx, nu, ng, equality):
        P = self.P
        opts = {
            "print_time": False,
            "structure_detection": "manual",
            "N": P.N, "nx": nx, "nu": nu, "ng": ng,
            "equality": equality,
            "fatrop": {
                "print_level": 0,
                "max_iter": P.max_iter,
                "tol": P.tol,
            },
        }
        self.solver = ca.nlpsol("shot", "fatrop", nlp, opts)

    ########################
    ### Packing helpers  ###
    ########################
    def unpack(self, w):
        N = self.P.N
        return {
            "X": w[self.idx["X"]].reshape((N + 1, NX)).T,
            "U": w[self.idx["U"]].reshape((N, NU)).T,
            "tf": float(w[self.i_tf]),
            "phi": float(w[self.idx["phi"][0]]),
            "vp": w[self.idx["vp"]].copy(),
            "zeta": float(w[self.idx["zeta"][0]]),
            "s_box": w[self.idx["s_box"]].copy(),
            "s_av": w[self.idx["s_av"]].copy(),
        }

    def pack(self, X, U, tf, phi, vp, zeta, s_box=None, s_av=None):
        N = self.P.N
        X, U = np.asarray(X, dtype=float), np.asarray(U, dtype=float)
        w = np.zeros(self.nw)
        w[self.idx["X"]] = X.T.ravel()
        w[self.idx["U"]] = U.T.ravel()
        w[self.idx["tf"]] = tf
        w[self.idx["U_last"]] = U[:, :N - 1].T.ravel()
        w[self.idx["phi"]] = phi
        w[self.idx["vp"]] = np.asarray(vp, dtype=float).ravel()
        w[self.idx["zeta"]] = zeta
        if s_box is not None:
            w[self.idx["s_box"]] = np.asarray(s_box, dtype=float).ravel()
        if s_av is not None:
            w[self.idx["s_av"]] = np.asarray(s_av, dtype=float).ravel()
        return w

    def make_params(self, x0, u_prev, ybar, hbar, q0, v0):
        """x0: robot state (x, y, vx, vy); u_prev: last input (a_x, a_y); (ybar, hbar): the virtual goal
        (shot_selector.virtual_goal); (q0, v0): puck state now."""
        return np.concatenate([
            np.asarray(x0, dtype=float), np.asarray(u_prev, dtype=float), [ybar, hbar],
            np.asarray(q0, dtype=float), np.asarray(v0, dtype=float),
        ])

    def init_slacks(self, w, p):
        """Set the slacks of an initial guess to its actual violations (feasible w.r.t. the soft rows)."""
        box, av = self.viol_fun(w, p)
        w = w.copy()
        w[self.idx["s_box"]] = np.maximum(np.array(box).ravel(), 0.0)
        w[self.idx["s_av"]] = np.maximum(np.array(av).ravel(), 0.0)
        return w

    ########################
    ### Initial guesses  ###
    ########################
    def warm_guess(self, sol, elapsed, xR, t_lo, t_hi):
        """A previous solution shifted by `elapsed` and interpolated onto the new grid (Section 7)."""
        N = self.P.N
        tf_new = float(np.clip(sol.tf - elapsed, t_lo, t_hi))
        t_old = np.linspace(0.0, sol.tf, N + 1)
        t_new = elapsed + np.linspace(0.0, tf_new, N + 1)
        X = np.array([np.interp(t_new, t_old, sol.X[i]) for i in range(NX)])
        X[:, 0] = xR
        mids = elapsed + (np.arange(N) + 0.5) * tf_new / N
        U = sol.U[:, zoh_interval(mids, sol.tf / N, N)]
        return self.pack(X, U, tf_new, sol.phi, sol.vp, sol.zeta)

    def cold_guess(self, xR, puck, ybar, tf):
        """Closed-form guess: virtual-goal aim, inverted impact and a Hermite cubic (Section 7)."""
        P = self.P
        N = P.N
        p_c, v_minus = puck.state_at(tf)
        goal = np.array([P.x_g, ybar])
        d = (goal - p_c) / np.linalg.norm(goal - p_c)
        n, v_n, phi = invert(P.sigma_init, d, v_minus, P.kappa)
        p_f = p_c - (P.r_R + P.r_p) * n
        v_f = (v_minus @ n + v_n) * n

        s = np.linspace(0.0, 1.0, N + 1)[None, :]
        p0, v0 = xR[0:2, None], xR[2:4, None]
        pf, vf = p_f[:, None], v_f[:, None]

        def hermite(s):
            pos = ((2 * s**3 - 3 * s**2 + 1) * p0 + (s**3 - 2 * s**2 + s) * tf * v0
                   + (3 * s**2 - 2 * s**3) * pf + (s**3 - s**2) * tf * vf)
            vel = ((6 * s**2 - 6 * s) * p0 + (3 * s**2 - 4 * s + 1) * tf * v0
                   + (6 * s - 6 * s**2) * pf + (3 * s**2 - 2 * s) * tf * vf) / tf
            acc = ((12 * s - 6) * p0 + (6 * s - 4) * tf * v0
                   + (6 - 12 * s) * pf + (6 * s - 2) * tf * vf) / tf**2
            return pos, vel, acc

        pos, vel, _ = hermite(s)
        X = np.vstack([pos, vel])
        X[:, 0] = xR
        _, vel_m, acc_m = hermite((np.arange(N)[None, :] + 0.5) / N)
        umax = P.u_max[:NU, None]
        U = np.clip(acc_m + P.gamma * vel_m, -umax, umax)

        _, vp, _ = impact(v_f, v_minus, n, P.kappa, P.m_p / P.m_R, P.mu, P.e)
        vp = np.array([max(vp[0], P.eps), vp[1]])
        zeta = float(np.clip(1.0 - P.gamma * (P.x_g - p_c[0]) / vp[0], ZETA_MIN, 1.0))
        return self.pack(X, U, tf, phi, vp, zeta)

    #############
    ### Solve ###
    #############
    def solve(self, w0, p, t_lo, t_hi, cold=False):
        P = self.P
        lbx, ubx = self.lbx.copy(), self.ubx.copy()
        lbx[self.i_tf], ubx[self.i_tf] = t_lo, t_hi
        w0 = np.clip(w0, lbx, ubx)

        t0 = time.perf_counter()
        res = None
        if np.abs(np.concatenate([w0, p])).max() < MAX_INPUT:  # np.max keeps NaN, so also False for NaN
            try:
                res = self.solver(x0=w0, p=p, lbx=lbx, ubx=ubx, lbg=self.lbg, ubg=self.ubg)
                stats = self.solver.stats()
                status, iters = str(stats["return_status"]), int(stats["iter_count"])
                converged = stats["success"] or iters >= P.max_iter  # a max_iter point still has to pass the checks below
            except RuntimeError:  # Fatrop raises instead of returning a status
                pass
        if res is None:  # an invalid solve, not a crash
            res = {"x": w0, "g": self.g_fun(w0, p), "f": self.f_fun(w0, p)}
            status, iters, converged = "failed", 0, False
        t_wall = time.perf_counter() - t0

        w = np.array(res["x"]).ravel()
        gv = np.array(res["g"]).ravel()
        cost = float(res["f"])
        viol = float(max(0.0, np.max(self.lbg - gv), np.max(gv - self.ubg)))
        parts = self.unpack(w)
        slack = float(max(np.max(parts["s_box"]), np.max(parts["s_av"]), 0.0))
        valid = bool(converged and np.isfinite(cost) and viol <= P.valid_tol and slack <= P.valid_tol)

        terms = dict(zip(self.term_names, np.array(self.terms_fun(w, p)).ravel()))
        X, phi = parts["X"], parts["phi"]
        p_c = X[0:2, -1] + (P.r_R + P.r_p) * np.array([np.cos(phi), np.sin(phi)])
        return Solution(
            w=w, X=X, U=parts["U"], tf=parts["tf"], phi=phi, p_c=p_c, vp=parts["vp"],
            zeta=parts["zeta"], cost=cost, status=status, iters=iters,
            t_wall=t_wall, valid=valid, viol=viol, slack=slack, cold=cold, terms=terms,
        )
