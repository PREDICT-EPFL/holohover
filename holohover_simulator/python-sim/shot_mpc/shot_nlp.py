"""
Inner problem P(B) for one fixed bounce sequence (Sections 2 to 5).

Built once with CasADi SX and solved with IPOPT; the sequence only enters
through the parameter y_bar (virtual goal), so one solver serves all of B_K.

Decision vector w = (X, U, t_f, phi, v+, zeta, s_box, s_av):
- X (4 x N+1), U (2 x N): robot states (x, y, vx, vy) and accelerations (a_x, a_y)
  on the grid t_f / N; the heading is left to the tracker
- phi: angle of the contact normal n = (cos phi, sin phi), left unwrapped
- v+: post-impact puck velocity, lifted with bound v+_x >= eps so every
  division by v+_x is safe at every IPOPT iterate
- zeta = 1 - gamma (x_g - x_c) / v+_x, lifted with bounds [ZETA_MIN, 1]: this is the
  reach constraint gamma (x_g - x_c) < v+_x, and T_B = -ln(zeta) / gamma is always defined
- s_box, s_av: L1 slacks of the soft table/own-half and puck-avoidance constraints

Parameters p = (x0_hat, u_prev, y_bar, q0, v0), with (q0, v0) the puck state at the
planning instant. The puck flies freely up to the impact, since t_f <= tau1 (Section 1).

Initial guesses w0 (Section 7): the previous solution shifted to now (warm), or a
closed-form guess from the virtual-goal aim and the inverted impact (cold).
"""

import atexit
import os
import shutil
import tempfile
import time
from contextlib import contextmanager
from dataclasses import dataclass

import numpy as np
import casadi as ca

from .model import GAMMA_EPS, flight_phi, free_flight, impact, invert, zoh_coefficients, zoh_interval

NX, NU = 4, 2  # robot state (x, y, vx, vy) and input (a_x, a_y)
ZETA_MIN = 1e-3  # floor of zeta for the strict reach inequality; T_B <= -ln(ZETA_MIN) / gamma
OK_STATUS = ("Solve_Succeeded", "Solved_To_Acceptable_Level")


@contextmanager
def jit_build_dir():
    """Run the JIT build in a private temp directory, removed at exit. CasADi writes
    jit_tmp*.c and tmp_casadi_compiler_shell*.{o,so} to the cwd and deletes them only when
    the solver is destroyed, so every killed run left them in python-sim/. Its
    jit_options "directory" does not help: CasADi 3.8.1 still writes the source to the cwd."""
    d = tempfile.mkdtemp(prefix="shot_nlp_jit_")
    atexit.register(shutil.rmtree, d, ignore_errors=True)
    cwd = os.getcwd()
    os.chdir(d)
    try:
        yield
    finally:
        os.chdir(cwd)


@dataclass
class Solution:
    w: np.ndarray
    X: np.ndarray
    U: np.ndarray
    tf: float
    phi: float
    p_c: np.ndarray  # puck centre at impact, exact by the contact equality
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
        zeta = ca.SX.sym("zeta") # reach margin, lifted with bounds [ZETA_MIN, 1] so T_B = -ln(zeta) / gamma is always defined
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
        q0 = ca.SX.sym("q0", 2) # puck position at the planning instant
        v0 = ca.SX.sym("v0", 2) # puck velocity at the planning instant
        p = ca.vertcat(x0, u_prev, ybar, q0, v0)
        self.np_ = p.numel() # numel = nummber of elements

        def puck_at(t):
            return free_flight(q0, v0, t, gam)

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
            pos, vel, u = X[0:2, k], X[2:4, k], U[:, k]
            add(X[:, k + 1] - ca.vertcat(pos + a * vel + b * u, e1 * vel + a * u), 0.0, 0.0)

        # Impact at t_f (Section 3)
        p_c, v_minus = puck_at(tf)
        n = ca.vertcat(ca.cos(ph), ca.sin(ph))
        add(X[0:2, N] - (p_c - (P.r_R + P.r_p) * n), 0.0, 0.0)
        v_n, vp_expr, vR_plus = impact(X[2:4, N], v_minus, n, P.kappa, P.m_p / P.m_R)
        add(vp - vp_expr, 0.0, 0.0)

        # Coarse post-impact model (Section 4)
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
            pp, _ = puck_at(k * dt)
            row = (d_min**2 - ca.sumsqr(X[0:2, k] - pp)) / (2 * d_min)
            add(row - s_av[k - 1], -ca.inf, 0.0)
            av_viol.append(row)

        g = ca.vertcat(*g)
        self.lbg = np.array(lbg, dtype=float)
        self.ubg = np.array(ubg, dtype=float)

        ############
        ### Cost ###
        ############
        # Terms A-E of Section 5, inputs normalised componentwise by their limits
        umax = P.u_max[:NU]
        ub_prev = u_prev / umax
        J_u, J_du = 0, 0
        for k in range(N):
            ub = U[:, k] / umax
            J_u += ca.sumsqr(ub)
            J_du += ca.sumsqr(ub - ub_prev)
            ub_prev = ub

        T_B = -ca.log(zeta) / gam if gam >= GAMMA_EPS else (P.x_g - x_c) / vp[0]
        e_aim = (y_c - ybar) + (P.x_g - x_c) * vp[1] / vp[0]

        terms = {
            "u": P.w_u * tf / N * J_u,
            "du": P.w_du * J_du,
            "tf": P.w_t * tf / P.t_ref,
            "T": P.w_T * T_B / P.T_ref,
            "aim": P.w_a * (e_aim / P.h_eff) ** 2,
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
        self.lbx[i_z], self.ubx[i_z] = ZETA_MIN, 1.0
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
                with jit_build_dir():
                    self._build_solvers(nlp, jit=True)
                self.jit = True
            except Exception as err:  # no compiler or compile error: interpreted fallback
                print(f"[shot_nlp] JIT build failed ({err}); falling back to the SX interpreter")
        if not self.jit:
            self._build_solvers(nlp, jit=False)
        self.t_build = time.perf_counter() - t_build
        if verbose:
            print(f"[shot_nlp] {self.nw} variables, {g.numel()} constraints, jit={self.jit}, "
                  f"build {self.t_build:.2f} s")

    def _build_solvers(self, nlp, jit):
        """One IPOPT solver per start type; they differ only in the iteration cap."""
        self.solver_warm = ca.nlpsol("shot_warm", "ipopt", nlp, self._opts(self.P.max_iter_warm, jit))
        self.solver_cold = ca.nlpsol("shot_cold", "ipopt", nlp, self._opts(self.P.max_iter_cold, jit))

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
            # No CasADi cleanup: its paths are relative to the build dir, which jit_build_dir removes
            opts.update({
                "jit": True,
                "compiler": "shell",
                "jit_cleanup": False,
                "jit_options": {"flags": list(P.jit_flags), "verbose": False, "cleanup": False},
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

    def make_params(self, x0, u_prev, ybar, q0, v0):
        """x0: robot state (x, y, vx, vy); u_prev: last input (a_x, a_y); (q0, v0): puck state now."""
        return np.concatenate([
            np.asarray(x0, dtype=float), np.asarray(u_prev, dtype=float), [ybar],
            np.asarray(q0, dtype=float), np.asarray(v0, dtype=float),
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

        _, vp, _ = impact(v_f, v_minus, n, P.kappa, P.m_p / P.m_R)
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
        X, phi = parts["X"], parts["phi"]
        p_c = X[0:2, -1] + (P.r_R + P.r_p) * np.array([np.cos(phi), np.sin(phi)])
        return Solution(
            w=w, X=X, U=parts["U"], tf=parts["tf"], phi=phi, p_c=p_c, vp=parts["vp"],
            zeta=parts["zeta"], cost=cost, status=status, iters=int(stats["iter_count"]),
            t_wall=t_wall, valid=valid, viol=viol, slack=slack, cold=cold, terms=terms,
        )
