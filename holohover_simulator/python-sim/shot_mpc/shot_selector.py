"""
Outer problem: choose the bounce sequence (Section 6), with the warm and cold
starts of Section 7.

Every B keeps its own warm start; one sequence is never warm-started from
another's solution. All times handed to the NLP are relative to the current
planning instant t_now; plans store their absolute start time t0.
"""

from dataclasses import dataclass, field

import numpy as np

from .bounce_geometry import sequences, label, virtual_goal_y
from .impact import invert, impact
from .shot_nlp import NU, ZETA_MIN


@dataclass
class Plan:
    key: tuple  # (s, k)
    label: str
    ybar: float
    t0: float  # absolute time of node 0
    sol: object  # shot_nlp.Solution
    score: float = np.inf  # selection criterion: the inner cost J*(B) (Section 6)
    t_spent: float = 0.0  # wall time of all solves that produced this plan

    @property
    def tf(self):
        return self.sol.tf

    @property
    def dt(self):
        return self.sol.tf / self.sol.U.shape[1]

    def remaining(self, t):
        return self.t0 + self.sol.tf - t

    def input_at(self, t):
        """ZOH input of the plan at absolute time t (last input after t_f)."""
        N = self.sol.U.shape[1]
        k = int(np.clip(np.floor((t - self.t0) / self.dt), 0, N - 1))
        return self.sol.U[:, k]

    def node_times(self):
        return self.t0 + np.linspace(0.0, self.sol.tf, self.sol.X.shape[1])


@dataclass
class SelectorLog:
    t: float
    outer: bool
    results: dict = field(default_factory=dict)  # label -> (valid, cost, score, tf, t_spent, iters, cold)
    chosen: str = None
    t_total: float = 0.0


class ShotSelector:
    def __init__(self, P, nlp):
        self.P = P
        self.nlp = nlp
        self.seqs = sequences(P.K)
        self.store = {}  # (s, k) -> Plan (last valid)
        self.candidates = {}  # (s, k) -> Plan of the latest solve, valid or not (visualisation)
        self.key_prev = None
        self.logs = []

    def reset(self):
        self.store.clear()
        self.candidates.clear()
        self.key_prev = None

    ######################
    ### Impact window  ###
    ######################
    def _tf_bounds(self, puck):
        """t_min <= t_f <= min(t_max, tau1): strike before the puck's next wall hit (Section 5)."""
        return self.P.t_min, min(self.P.t_max, puck.tau1)

    def _contact_possible(self, puck, t_lo, t_hi):
        """
        Necessary condition for any shot: with the whole robot in its own half, contact
        needs the puck at x <= x_mid + r_p. Before tau1 x(t) is monotone, so checking
        both ends is exact. Skips the costly infeasible solves (puck gone).
        """
        x_lo = puck.state_at(t_lo)[0][0]
        x_hi = puck.state_at(t_hi)[0][0]
        return min(x_lo, x_hi) <= self.P.x_mid + self.P.r_p

    def _params(self, xR, u_prev, ybar, puck):
        return self.nlp.make_params(xR, u_prev, ybar, puck.q, puck.v)

    ##################
    ### Warm start ###
    ##################
    def _shift(self, plan, t_now, xR, t_lo, t_hi):
        """Shift a stored plan to t_now and interpolate it onto the new grid (Section 7)."""
        N = self.P.N
        sol = plan.sol
        elapsed = t_now - plan.t0
        tf_new = float(np.clip(sol.tf - elapsed, t_lo, t_hi))
        t_old = np.linspace(0.0, sol.tf, N + 1)
        t_new = elapsed + np.linspace(0.0, tf_new, N + 1)
        X = np.array([np.interp(t_new, t_old, sol.X[i]) for i in range(sol.X.shape[0])])
        X[:, 0] = xR
        dt_old = sol.tf / N
        mids = elapsed + (np.arange(N) + 0.5) * tf_new / N
        idx = np.clip(np.floor(mids / dt_old).astype(int), 0, N - 1)
        U = sol.U[:, idx]
        return self.nlp.pack(X, U, tf_new, sol.phi, sol.vp, sol.zeta)

    ##################
    ### Cold start ###
    ##################
    def _cold_guess(self, xR, puck, ybar, tf):
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
        return self.nlp.pack(X, U, tf, phi, vp, zeta)

    ##############
    ### Solves ###
    ##############
    def _solve_key(self, key, t_now, xR, u_prev, puck, allow_cold):
        """Solve P(B) for one B = (s, k): warm start if stored, else cold start with t_f seeds."""
        P = self.P
        s, k = key
        ybar = virtual_goal_y(s, k, P.w)
        t_lo, t_hi = self._tf_bounds(puck)
        if t_hi < t_lo or not self._contact_possible(puck, t_lo, t_hi):
            return None

        best = None
        t_spent = 0.0  # all solves for this key, including every cold seed
        p = self._params(xR, u_prev, ybar, puck)
        plan = self.store.get(key)
        if plan is not None:
            w0 = self._shift(plan, t_now, xR, t_lo, t_hi)
            sol = self.nlp.solve(self.nlp.init_slacks(w0, p), p, t_lo, t_hi, cold=False)
            best = sol
            t_spent += sol.t_wall

        if (best is None or not best.valid) and allow_cold:
            n_seeds = max(1, P.n_tf_seeds)
            seeds = np.linspace(t_lo, t_hi, n_seeds + 2)[1:-1] if n_seeds > 1 else [0.5 * (t_lo + t_hi)]
            for tf0 in seeds:
                w0 = self.nlp.init_slacks(self._cold_guess(xR, puck, ybar, tf0), p)
                sol = self.nlp.solve(w0, p, t_lo, t_hi, cold=True)
                t_spent += sol.t_wall
                if best is None or (sol.valid and (not best.valid or sol.cost < best.cost)):
                    best = sol

        if best is None:
            return None
        new_plan = Plan(key=key, label=label(s, k), ybar=ybar, t0=t_now, sol=best, t_spent=t_spent)
        if best.valid:
            self.store[key] = new_plan
        return new_plan

    def update(self, t_now, xR, u_prev, puck, outer=True, commit_key=None, allow_cold=True):
        """
        One planning update. With outer=True every sequence is solved and the one with
        the lowest inner cost J*(B) is selected; otherwise only the current (or committed)
        sequence is re-solved. xR = (x, y, vx, vy), u_prev = (a_x, a_y), puck: PuckFlight.
        Returns the chosen valid Plan or None.
        """
        log = SelectorLog(t=t_now, outer=outer)
        if commit_key is not None:
            keys = [commit_key]
        elif outer or self.key_prev is None:
            keys = list(self.seqs)
        else:
            keys = [self.key_prev]

        chosen = None
        for key in keys:
            plan = self._solve_key(key, t_now, xR, u_prev, puck, allow_cold)
            if plan is None:
                self.candidates.pop(key, None)
                continue
            sol = plan.sol
            plan.score = sol.cost
            self.candidates[key] = plan
            log.results[plan.label] = (sol.valid, sol.cost, plan.score, sol.tf, plan.t_spent, sol.iters, sol.cold)
            log.t_total += plan.t_spent
            if sol.valid and (chosen is None or plan.score < chosen.score):
                chosen = plan

        if chosen is not None:
            self.key_prev = chosen.key
            log.chosen = chosen.label
        self.logs.append(log)
        return chosen
