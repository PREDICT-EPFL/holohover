"""
Inner problem P(B) for one fixed bounce sequence (Sections 3, 4, 5 and 8).

Built once with CasADi SX and solved with IPOPT; the sequence only enters
through the parameter y_bar (virtual goal), so one solver serves all of B_K.

Decision vector w = (X, U, t_f, phi, v+, zeta, s_box, s_av):
- X (6 x N+1), U (3 x N): robot states and accelerations on the grid t_f / N
- phi: angle of the contact normal n = (cos phi, sin phi), left unwrapped
- v+: post-impact puck velocity, lifted with bound v+_x >= eps so every
  division by v+_x is safe at every IPOPT iterate
- zeta = 1 - gamma (x_g - x_c) / v+_x, lifted with bounds [eta, 1]: this is the
  reach margin, and T_B = -ln(zeta) / gamma is always defined
- s_box, s_av: L1 slacks of the soft table/own-half and puck-avoidance constraints

Parameters p = (x0_hat, u_prev, y_bar, theta_ref, node puck windows, impact window).
A puck window is (qx, qy, vx, vy, tau_start); v1 uses window 0 everywhere, but
the layout already allows pre-impact bounces (Section 7).
"""

import time
from dataclasses import dataclass

import numpy as np
import casadi as ca

from .puck_predictor import GAMMA_EPS, flight_phi, flight_decay
from .impact import impact

NX, NU, NWIN = 6, 3, 5 # state and input dimensions, number of puck window parameters
OK_STATUS = ("Solve_Succeeded", "Solved_To_Acceptable_Level")


def zoh_coefficients(dt, gamma, dt_max=None):
    """
    (e1, a, b) of the exact ZOH map p+ = p + a v + b u, v+ = e1 v + a u (Section 3).

    a = (1 - e^{-x}) / gamma and b = (dt - a) / gamma with x = gamma dt. The closed
    form of b cancels catastrophically for small x, so a Taylor series is used when
    gamma * dt_max < 0.1 (truncation error < 1e-14). dt_max bounds a symbolic dt.
    """
    if gamma < GAMMA_EPS:
        return 1 + 0 * dt, dt, 0.5 * dt**2
    e1 = flight_decay(dt, gamma)
    x_max = gamma * (dt if dt_max is None else dt_max)
    if x_max < 0.1:
        x = gamma * dt
        fact = [1.0]
        for j in range(1, 11):
            fact.append(fact[-1] * j)
        phi1 = sum((-1) ** j / fact[j + 1] * x**j for j in range(8))  # (1 - e^-x) / x
        phi2 = sum((-1) ** j / fact[j + 2] * x**j for j in range(8))  # (x - 1 + e^-x) / x^2
        return e1, dt * phi1, dt**2 * phi2
    a = (1 - e1) / gamma
    b = (dt - a) / gamma
    return e1, a, b


def discrete_matrices(dt, gamma):
    """Numeric A_d, B_d of the robot double integrator with viscous friction."""
    e1, a, b = zoh_coefficients(dt, gamma)
    I3 = np.eye(3)
    A = np.block([[I3, a * I3], [np.zeros((3, 3)), e1 * I3]])
    B = np.vstack([b * I3, a * I3])
    return A, B


@dataclass
class Solution:
    w: np.ndarray
    X: np.ndarray
    U: np.ndarray
    tf: float
    phi: float
    vp: np.ndarray
    zeta: float
    cost: float
    status: str
    iters: int
    t_wall: float
    valid: bool
    viol: float
    slack: float
    cold: bool
    terms: dict


class ShotNLP:
    def __init__(self, P, jit=None, verbose=True):
        self.P = P
        N = P.N
        gam = P.gamma
        jit = P.jit if jit is None else jit

        ###########################
        ### Decision variables  ###
        ###########################
        X = ca.SX.sym("X", NX, N + 1)
        U = ca.SX.sym("U", NU, N)
        tf = ca.SX.sym("tf")
        ph = ca.SX.sym("phi") # angle of the contact normal n = (cos phi, sin phi)
        vp = ca.SX.sym("vp", 2)
        zeta = ca.SX.sym("zeta") # reach margin, lifted with bounds [eta, 1] so T_B = -ln(zeta) / gamma is always defined
        s_box = ca.SX.sym("s_box", N) # L1 slacks of the soft table/own-half constraints
        s_av = ca.SX.sym("s_av", N - 1) # L1 slacks of the soft puck-avoidance constraints

        blocks = [("X", ca.vec(X)), ("U", ca.vec(U)), ("tf", tf), ("phi", ph),
                  ("vp", vp), ("zeta", zeta), ("s_box", s_box), ("s_av", s_av)]
        self.off, o = {}, 0
        for name, b in blocks:
            self.off[name] = (o, o + b.numel())
            o += b.numel()
        self.nw = o
        w = ca.vertcat(*[b for _, b in blocks])

        ##################
        ### Parameters ###
        ##################
        x0 = ca.SX.sym("x0", NX)
        u_prev = ca.SX.sym("u_prev", NU)
        ybar = ca.SX.sym("ybar")
        th_ref = ca.SX.sym("theta_ref")
        win = ca.SX.sym("win", NWIN, N - 1) # puck window parameters for nodes 1..N-1
        win_i = ca.SX.sym("win_i", NWIN) # puck window parameters for the impact node N
        p = ca.vertcat(x0, u_prev, ybar, th_ref, ca.vec(win), win_i)
        self.np_ = p.numel() # numel = nummber of elements

        def puck_at(wv, t):
            q, v, ts = wv[0:2], wv[2:4], wv[4]
            return q + v * flight_phi(t - ts, gam), v * flight_decay(t - ts, gam)

        ###################
        ### Constraints ###
        ###################
        g, lbg, ubg = [], [], []

        def add(expr, lo, hi):
            g.append(expr)
            lbg.extend([lo] * expr.numel())
            ubg.extend([hi] * expr.numel())

        dt = tf / N
        e1, a, b = zoh_coefficients(dt, gam, dt_max=P.t_max / N)

        # Robot dynamics (nodes 0..N-1)
        add(X[:, 0] - x0, 0.0, 0.0)
        for k in range(N):
            pos, vel, u = X[0:3, k], X[3:6, k], U[:, k]
            add(X[:, k + 1] - ca.vertcat(pos + a * vel + b * u, e1 * vel + a * u), 0.0, 0.0)

        # Impact at t_f (Section 4)
        p_c, v_minus = puck_at(win_i, tf)
        n = ca.vertcat(ca.cos(ph), ca.sin(ph))
        add(X[0:2, N] - (p_c - (P.r_R + P.r_p) * n), 0.0, 0.0)
        v_n, vp_expr, vR_plus = impact(X[3:5, N], v_minus, n, P.kappa, P.m_p / P.m_R)
        add(vp - vp_expr, 0.0, 0.0)

        # Coarse post-impact model (Section 5)
        x_c, y_c = p_c[0], p_c[1]
        add(zeta * vp[0] - (vp[0] - gam * (P.x_g - x_c)), 0.0, 0.0)
        add(v_n, P.v_min, ca.inf)
        aim = (y_c - ybar) * vp[0] + (P.x_g - x_c) * vp[1]
        add(aim - P.h_eff * vp[0], -ca.inf, 0.0)
        add(-aim - P.h_eff * vp[0], -ca.inf, 0.0)

        # Soft table limits, whole robot in own half (nodes 1..N)
        x_hi = P.x_mid - P.r_R
        y_hi = 0.5 * P.W - P.r_R
        box_viol = []
        # Post-impact clearance (shares node N's slack): t_clear after impact, flying with
        # its recoil velocity v_R+, the robot must still be off the side walls and our end
        # wall. Without it a strike driven into a nearby wall hits the wall right after the
        # puck; the simulator's 10 ms step then resolves the wall bounce first and spoils the shot.
        p_clear = X[0:2, N] + vR_plus * flight_phi(P.t_clear, gam)
        clear_rows = ca.vertcat(P.r_R - p_clear[0], p_clear[1] - y_hi, -y_hi - p_clear[1])
        for k in range(1, N + 1):
            rows = ca.vertcat(X[0, k] - x_hi, P.r_R - X[0, k], X[1, k] - y_hi, -y_hi - X[1, k])
            if k == N and P.t_clear > 0:
                rows = ca.vertcat(rows, clear_rows)
            add(rows - s_box[k - 1], -ca.inf, 0.0)
            box_viol.append(ca.mmax(rows))

        # Soft puck avoidance before impact (nodes 1..N-1). The margin shrinks
        # linearly to 0 at impact: a fixed delta contradicts the contact equality
        # for short t_f, while delta (N-k)/N is met by a straight final approach
        # at v_min whenever t_f >= delta / v_min.
        av_viol = []
        for k in range(1, N):
            d_min = P.r_R + P.r_p + P.delta * (N - k) / N
            pp, _ = puck_at(win[:, k - 1], k * dt)
            row = (d_min**2 - ca.sumsqr(X[0:2, k] - pp)) / (2 * d_min)
            add(row - s_av[k - 1], -ca.inf, 0.0)
            av_viol.append(row)

        g = ca.vertcat(*g)
        self.lbg = np.array(lbg, dtype=float)
        self.ubg = np.array(ubg, dtype=float)

        ############
        ### Cost ###
        ############
        umax = P.u_max
        ub_prev = u_prev / umax
        J_u, J_th, J_du = 0, 0, 0
        for k in range(N):
            ub = U[:, k] / umax
            J_u += sum(P.R[i] * ub[i] ** 2 for i in range(NU))
            J_du += sum(P.R_delta[i] * (ub[i] - ub_prev[i]) ** 2 for i in range(NU))
            J_th += P.w_theta * ((X[2, k] - th_ref) / P.theta_scale) ** 2
            J_th += P.w_omega * (X[5, k] / P.omega_scale) ** 2
            ub_prev = ub
        J_run = tf / N * (J_u + J_th)

        T_B = -ca.log(zeta) / gam if gam >= GAMMA_EPS else (P.x_g - x_c) / vp[0]
        e_aim = (y_c - ybar) + (P.x_g - x_c) * vp[1] / vp[0]
        sin2 = (n[0] * vp[1] - n[1] * vp[0]) ** 2 / (vp[0] ** 2 + vp[1] ** 2)

        terms = {
            "run": J_run,
            "du": J_du,
            "tf": P.w_t * tf / P.t_ref,
            "T": P.w_T * T_B / P.T_ref,
            "aim": P.w_a * (e_aim / P.h_eff) ** 2,
            "angle": P.w_n * sin2,
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
        o0, o1 = self.off["U"]
        self.lbx[o0:o1] = np.tile(-umax, N)
        self.ubx[o0:o1] = np.tile(umax, N)
        self.i_tf = self.off["tf"][0]
        i_vp = self.off["vp"][0]
        self.lbx[i_vp] = P.eps
        i_z = self.off["zeta"][0]
        self.lbx[i_z], self.ubx[i_z] = P.eta, 1.0
        for name in ("s_box", "s_av"):
            o0, o1 = self.off[name]
            self.lbx[o0:o1] = 0.0

        ##############
        ### Solver ###
        ##############
        nlp = {"x": w, "p": p, "f": J, "g": g}
        t_build = time.perf_counter()
        self.jit = False
        if jit:
            try:
                self.solver_warm = ca.nlpsol("shot_warm", "ipopt", nlp, self._opts(P.max_iter_warm, True))
                self.solver_cold = ca.nlpsol("shot_cold", "ipopt", nlp, self._opts(P.max_iter_cold, True))
                self.jit = True
            except Exception as err:  # no compiler or compile error: interpreted fallback
                print(f"[shot_nlp] JIT build failed ({err}); falling back to the SX interpreter")
        if not self.jit:
            self.solver_warm = ca.nlpsol("shot_warm", "ipopt", nlp, self._opts(P.max_iter_warm, False))
            self.solver_cold = ca.nlpsol("shot_cold", "ipopt", nlp, self._opts(P.max_iter_cold, False))
        self.t_build = time.perf_counter() - t_build
        if verbose:
            print(f"[shot_nlp] {self.nw} variables, {g.numel()} constraints, jit={self.jit}, "
                  f"build {self.t_build:.2f} s")

    def _opts(self, max_iter, jit):
        P = self.P
        opts = {
            "print_time": False,
            "ipopt": {
                "print_level": 0,
                "sb": "yes",
                "max_iter": max_iter,
                "tol": P.tol,
                "acceptable_tol": P.acceptable_tol,
                "acceptable_iter": 5,
            },
        }
        if jit:
            opts.update({
                "jit": True,
                "compiler": "shell",
                "jit_options": {"flags": list(P.jit_flags), "verbose": False},
            })
        return opts

    ########################
    ### Packing helpers  ###
    ########################
    def _block(self, w, name):
        o0, o1 = self.off[name]
        return w[o0:o1]

    def unpack(self, w):
        N = self.P.N
        return {
            "X": self._block(w, "X").reshape((N + 1, NX)).T,
            "U": self._block(w, "U").reshape((N, NU)).T,
            "tf": float(self._block(w, "tf")[0]),
            "phi": float(self._block(w, "phi")[0]),
            "vp": self._block(w, "vp").copy(),
            "zeta": float(self._block(w, "zeta")[0]),
            "s_box": self._block(w, "s_box").copy(),
            "s_av": self._block(w, "s_av").copy(),
        }

    def pack(self, X, U, tf, phi, vp, zeta, s_box=None, s_av=None):
        N = self.P.N
        s_box = np.zeros(N) if s_box is None else s_box
        s_av = np.zeros(N - 1) if s_av is None else s_av
        return np.concatenate([
            np.asarray(X, dtype=float).T.ravel(),
            np.asarray(U, dtype=float).T.ravel(),
            [tf, phi], np.asarray(vp, dtype=float).ravel(), [zeta],
            np.asarray(s_box, dtype=float).ravel(), np.asarray(s_av, dtype=float).ravel(),
        ])

    def make_params(self, x0, u_prev, ybar, theta_ref, node_windows, impact_window):
        """node_windows: (N-1, 5) window data for nodes 1..N-1; impact_window: (5,)."""
        return np.concatenate([
            np.asarray(x0, dtype=float), np.asarray(u_prev, dtype=float), [ybar, theta_ref],
            np.asarray(node_windows, dtype=float).ravel(), np.asarray(impact_window, dtype=float),
        ])

    def init_slacks(self, w, p):
        """Set the slacks of an initial guess to its actual violations (feasible w.r.t. the soft rows)."""
        box, av = self.viol_fun(w, p)
        w = w.copy()
        o0, o1 = self.off["s_box"]
        w[o0:o1] = np.maximum(np.array(box).ravel(), 0.0)
        o0, o1 = self.off["s_av"]
        w[o0:o1] = np.maximum(np.array(av).ravel(), 0.0)
        return w

    #############
    ### Solve ###
    #############
    def solve(self, w0, p, t_lo, t_hi, cold=False):
        P = self.P
        lbx, ubx = self.lbx.copy(), self.ubx.copy()
        lbx[self.i_tf], ubx[self.i_tf] = t_lo, t_hi
        w0 = np.clip(w0, lbx, ubx)
        solver = self.solver_cold if cold else self.solver_warm

        t0 = time.perf_counter()
        res = solver(x0=w0, p=p, lbx=lbx, ubx=ubx, lbg=self.lbg, ubg=self.ubg)
        t_wall = time.perf_counter() - t0

        stats = solver.stats()
        status = stats["return_status"]
        w = np.array(res["x"]).ravel()
        gv = np.array(res["g"]).ravel()
        cost = float(res["f"])
        viol = float(max(0.0, np.max(self.lbg - gv), np.max(gv - self.ubg)))
        parts = self.unpack(w)
        slack = float(max(np.max(parts["s_box"]), np.max(parts["s_av"]), 0.0))
        converged = status in OK_STATUS or status == "Maximum_Iterations_Exceeded"
        valid = bool(converged and np.isfinite(cost) and viol <= P.valid_tol and slack <= P.valid_tol)

        terms = dict(zip(self.term_names, np.array(self.terms_fun(w, p)).ravel()))
        return Solution(
            w=w, X=parts["X"], U=parts["U"], tf=parts["tf"], phi=parts["phi"], vp=parts["vp"],
            zeta=parts["zeta"], cost=cost, status=status, iters=int(stats["iter_count"]),
            t_wall=t_wall, valid=valid, viol=viol, slack=slack, cold=cold, terms=terms,
        )
