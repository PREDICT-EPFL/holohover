"""
Outer problem: choose the bounce sequence (Section 6), with the warm and cold
starts of Section 7.

A sequence B = (s, k) has first wall s (+1 = L, -1 = R) and k alternating
side-wall bounces; it scores iff the unfolded straight shot hits the mirrored
goal at y_bar = 2 s k w (Section 4).

Every B keeps its own warm start; one sequence is never warm-started from
another's solution. All times handed to the NLP are relative to the current
planning instant t_now; plans store their absolute start time t0.
"""

from dataclasses import dataclass, field

import numpy as np

from .model import zoh_coefficients, zoh_interval


#########################
### Bounce sequences  ###
#########################
def sequences(K):
    """All sequences with at most K bounces, direct shot first: [(s, k)].
    for K=2 it return  [(0, 0), (1, 1), (-1, 1), (1, 2), (-1, 2)]"""
    out = [(0, 0)]
    for k in range(1, K + 1):
        for s in (+1, -1):
            out.append((s, k))
    return out


def label(s, k):
    """Return a string label for the sequence (s, k).
    s = +1: first bounce on L, s = -1: first bounce on R, k = 0: direct shot."""
    if k == 0:
        return "direct"
    return "".join("L" if s * (-1) ** j > 0 else "R" for j in range(k))


def virtual_goal_y(s, k, w):
    """y of the goal image after k bounces starting on wall s, e.g. +2w for L (s = +1, k = 1)."""
    return 2 * s * k * w


#############
### Plan  ###
#############
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
        """ZOH input (a_x, a_y) of the plan at absolute time(s) t (last input after t_f)."""
        k = zoh_interval(np.asarray(t, dtype=float) - self.t0, self.dt, self.sol.U.shape[1])
        return self.sol.U[:, k]

    def state_at(self, times, gamma):
        """Plan states (x, y, vx, vy) at an array of absolute times, propagated exactly
        from the preceding node with its ZOH input."""
        s = np.asarray(times, dtype=float) - self.t0
        k = zoh_interval(s, self.dt, self.sol.U.shape[1])
        e1, a, b = zoh_coefficients(s - k * self.dt, gamma, dt_max=self.dt)
        X, U = self.sol.X[:, k], self.sol.U[:, k]
        # Exact ZOH step from node k: position p + a v + b u, velocity e1 v + a u
        return np.vstack([X[0:2] + a * X[2:4] + b * U, e1 * X[2:4] + a * U])


@dataclass
class SelectorLog:
    t: float
    outer: bool
    results: dict = field(default_factory=dict)  # label -> (valid, cost, score, tf, t_spent, iters, cold)
    chosen: str = None
    t_total: float = 0.0


################
### Selector ###
################
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

    def _contact_possible(self, puck, t_lo, t_hi):
        """
        Necessary condition for any shot: with the whole robot in its own half, contact
        needs the puck at x <= x_mid + r_p. Before tau1 x(t) is monotone, so checking
        both ends is exact. Skips the costly infeasible solves (puck gone).
        """
        x_lo = puck.state_at(t_lo)[0][0]
        x_hi = puck.state_at(t_hi)[0][0]
        return min(x_lo, x_hi) <= self.P.x_mid + self.P.r_p

    def _solve_key(self, key, t_now, xR, u_prev, puck, allow_cold):
        """Solve P(B) for one B = (s, k): warm start if stored, else cold start with t_f seeds."""
        P, nlp = self.P, self.nlp
        s, k = key
        ybar = virtual_goal_y(s, k, P.w)
        # t_min <= t_f <= min(t_max, tau1): strike before the puck's next wall hit (Section 5)
        t_lo, t_hi = P.t_min, min(P.t_max, puck.tau1)
        if t_hi < t_lo or not self._contact_possible(puck, t_lo, t_hi):
            return None

        best = None
        t_spent = 0.0  # all solves for this key, including every cold seed
        p = nlp.make_params(xR, u_prev, ybar, puck.q, puck.v)
        plan = self.store.get(key)
        if plan is not None:
            w0 = nlp.warm_guess(plan.sol, t_now - plan.t0, xR, t_lo, t_hi)
            sol = nlp.solve(nlp.init_slacks(w0, p), p, t_lo, t_hi, cold=False)
            best = sol
            t_spent += sol.t_wall

        if (best is None or not best.valid) and allow_cold:
            n_seeds = max(1, P.n_tf_seeds)
            seeds = np.linspace(t_lo, t_hi, n_seeds + 2)[1:-1] if n_seeds > 1 else [0.5 * (t_lo + t_hi)]
            for tf0 in seeds:
                w0 = nlp.init_slacks(nlp.cold_guess(xR, puck, ybar, tf0), p)
                sol = nlp.solve(w0, p, t_lo, t_hi, cold=True)
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
