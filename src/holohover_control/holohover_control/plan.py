"""
A plan as the controller follows it: a planner's chosen candidate (holohover_msgs
HolohoverPlanCandidate), table frame. It holds the states (x, y, vx, vy) at the N + 1 nodes
t0 + k tf / N up to the planned impact, and the acceleration (a_x, a_y) held on each of the N
intervals. A plan has no heading.

The model is the planner's (holohover_planner shot_mpc/model.py): per axis a double integrator
with viscous friction gamma, discretised exactly with zero-order-hold inputs over a step dt:
    p+ = p + a v + b u,   v+ = e1 v + a u,   e1 = exp(-gamma dt), a = (1 - e1) / gamma, b = (dt - a) / gamma
The functions work on numpy arrays and on CasADi symbols alike.
"""

from dataclasses import dataclass

import numpy as np
import casadi as ca

GAMMA_EPS = 1e-9


def _symbolic(*xs):
    return any(isinstance(x, (ca.SX, ca.MX, ca.DM)) for x in xs)


def zoh_coefficients(dt, gamma):
    """(e1, a, b) of the exact ZOH map over dt."""
    if gamma < GAMMA_EPS:
        return 1 + 0 * dt, dt, 0.5 * dt**2
    e1 = ca.exp(-gamma * dt) if _symbolic(dt) else np.exp(-gamma * dt)
    a = (1 - e1) / gamma
    b = (dt - a) / gamma
    return e1, a, b


def zoh_interval(s, dt, N):
    """Index of the ZOH interval holding time s (since node 0) on a grid of N intervals of
    length dt; times before the grid get the first interval, at or after its end the last."""
    return np.clip(np.floor(s / dt).astype(int), 0, N - 1)


def discrete_matrices(dt, gamma, axes=2):
    """A_d, B_d of the state (p, v) on `axes` axes: x+ = A_d x + B_d u, A_d = [[I, a I], [0, e1 I]],
    B_d = [[b I], [a I]]. Numpy for a numeric dt, CasADi SX for a symbolic one."""
    e1, a, b = zoh_coefficients(dt, gamma)
    if _symbolic(dt):
        I = ca.SX.eye(axes)
        return ca.blockcat([[I, a * I], [ca.SX(axes, axes), e1 * I]]), ca.vertcat(b * I, a * I)
    I = np.eye(axes)
    return np.block([[I, a * I], [np.zeros((axes, axes)), e1 * I]]), np.vstack([b * I, a * I])


@dataclass
class Plan:
    label: str
    t0: float  # absolute time of node 0
    tf: float  # time from node 0 to the impact
    X: np.ndarray  # (N + 1, 4) node states (x, y, vx, vy)
    U: np.ndarray  # (N, 2) interval accelerations (a_x, a_y)

    @classmethod
    def from_candidate(cls, c):
        """From a planner's candidate without ROS (holohover_planner shot_planner.Candidate: label, t0,
        tf, states, acc), as python-sim hands it over."""
        return cls(c.label, c.t0, c.tf, np.asarray(c.states, dtype=float), np.asarray(c.acc, dtype=float))

    @property
    def N(self):
        return len(self.U)

    @property
    def dt(self):
        return self.tf / self.N

    def remaining(self, t):
        """Time left to the planned impact."""
        return self.t0 + self.tf - t

    def input_at(self, t):
        """ZOH acceleration (a_x, a_y) at absolute time(s) t, (2,) or (2, n) (the last one after the impact)."""
        return self.U[zoh_interval(np.asarray(t, dtype=float) - self.t0, self.dt, self.N)].T

    def state_at(self, times, gamma):
        """States (x, y, vx, vy) at an array of absolute times, (4, n), propagated exactly from the
        preceding node with its ZOH input."""
        s = np.asarray(times, dtype=float) - self.t0
        k = zoh_interval(s, self.dt, self.N)
        e1, a, b = zoh_coefficients(s - k * self.dt, gamma)
        X, U = self.X[k].T, self.U[k].T
        return np.vstack([X[0:2] + a * X[2:4] + b * U, e1 * X[2:4] + a * U])
