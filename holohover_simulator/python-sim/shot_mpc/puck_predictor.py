"""
Closed-form puck prediction with linear drag, up to the first wall hit (Section 1).

All quantities are in the planner frame. Until its first wall hit tau1 the puck
is in free flight from the measured state (q, v):
    p(t) = q + v * phi(t),   v(t) = v * exp(-gamma t)
with phi(t) = (1 - exp(-gamma t)) / gamma (-> t for gamma -> 0).
The planner only strikes before tau1. bounce_path chains these flights through
pure wall reflections (exact for this simulator) to draw the puck path.
"""

from dataclasses import dataclass

import numpy as np
import casadi as ca

GAMMA_EPS = 1e-9


def _exp(x):
    if isinstance(x, (ca.SX, ca.MX, ca.DM)):
        return ca.exp(x)
    return np.exp(x)


def flight_phi(t, gamma):
    """Distance factor of the damped free flight; works on floats, arrays and CasADi."""
    if gamma < GAMMA_EPS:
        return t
    return (1 - _exp(-gamma * t)) / gamma


def flight_decay(t, gamma):
    if gamma < GAMMA_EPS:
        return 1 + 0 * t
    return _exp(-gamma * t)


def free_flight(q, v, t, gamma):
    """Position and velocity at time t of a puck in free flight from (q, v)."""
    return q + v * flight_phi(t, gamma), v * flight_decay(t, gamma)


def hit_time(c, q, v, gamma):
    """Time for a coordinate at q moving with v to reach c (inf if it never does)."""
    d = c - q
    if v == 0.0 or d * v <= 0.0:
        return np.inf
    if gamma < GAMMA_EPS:
        return d / v
    arg = 1.0 - gamma * d / v
    if arg <= 0.0:
        return np.inf  # the puck stops before reaching the wall
    return -np.log(arg) / gamma


@dataclass
class PuckFlight:
    """Free flight from the puck state (q, v) at time 0, valid until the first wall hit tau1."""

    q: np.ndarray  # (2,) position at time 0
    v: np.ndarray  # (2,) velocity at time 0
    tau1: float  # first wall hit, inf if the puck stops before any wall
    wall: str  # wall hit at tau1: 'L', 'R', 'near' (our end), 'far' (opponent end); None if none
    gamma: float

    def state_at(self, t):
        return free_flight(self.q, self.v, t, self.gamma)


def predict(q0, v0, P):
    """
    Puck flight from the measured state. Side walls are at y = +-w, our end wall at
    x = r_p and the opponent end wall at x = x_g (puck centre); the puck moves toward
    at most one side wall and one end wall, so tau1 = min(tau_side, tau_end).
    """
    # A collision's position correction can leave the puck marginally outside the effective walls
    q = np.array([np.clip(q0[0], P.r_p, P.x_g), np.clip(q0[1], -P.w, P.w)], dtype=float)
    v = np.array(v0, dtype=float)
    hits = [
        (hit_time(P.w, q[1], v[1], P.gamma), "L"),
        (hit_time(-P.w, q[1], v[1], P.gamma), "R"),
        (hit_time(P.x_g, q[0], v[0], P.gamma), "far"),
        (hit_time(P.r_p, q[0], v[0], P.gamma), "near"),
    ]
    tau1, wall = min(hits, key=lambda hit: hit[0])
    return PuckFlight(q=q, v=v, tau1=tau1, wall=wall if np.isfinite(tau1) else None, gamma=P.gamma)


def bounce_path(q0, v0, P, t_end, max_hits=30):
    """
    Vertices of the free puck path from now to t_end, for display and tests: the start,
    every wall hit, and the position at t_end. A side wall flips v_y, an end wall v_x;
    crossing an end wall inside the mouth (|y| <= h, the simulator's scoring rule on
    the puck centre) ends the path there.

    Returns (points (n, 2), hits): hits[j] names vertex j + 1, one of 'L', 'R', 'near',
    'far', 'goal_for' or 'goal_against'. The last vertex has no entry unless it is a goal.
    """
    flight = predict(q0, v0, P)
    points, hits = [flight.q], []
    for _ in range(max_hits):
        if flight.tau1 > t_end:
            break
        t_end -= flight.tau1
        q, v = flight.state_at(flight.tau1)
        points.append(q)
        if flight.wall in ("near", "far") and abs(q[1]) <= P.h:
            hits.append("goal_for" if flight.wall == "far" else "goal_against")
            return np.array(points), hits
        hits.append(flight.wall)
        v[1 if flight.wall in ("L", "R") else 0] *= -1
        flight = predict(q, v, P)
    points.append(flight.state_at(t_end)[0])
    return np.array(points), hits
