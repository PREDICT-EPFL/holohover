"""
Prediction model shared by the planner, the tracker and the display, in the planner frame.

- Damped free flight (Section 1): from (q, v) the puck moves as
      p(t) = q + v * phi(t),   v(t) = v * exp(-gamma t)
  with phi(t) = (1 - exp(-gamma t)) / gamma (-> t for gamma -> 0).
- Exact ZOH map of the robot double integrator with viscous friction (Section 2).
- Frictionless impulsive robot-puck impact (Section 3).
- Puck prediction up to the first wall hit tau1, and the drawn puck paths.

The flight, ZOH and impact functions use only elementwise operations, so they work
on numpy arrays and on CasADi symbols alike.
"""

from dataclasses import dataclass

import numpy as np
import casadi as ca

GAMMA_EPS = 1e-9


def _symbolic(*xs):
    return any(isinstance(x, (ca.SX, ca.MX, ca.DM)) for x in xs)


def _exp(x):
    return ca.exp(x) if _symbolic(x) else np.exp(x)


def _vec2(x, y):
    return ca.vertcat(x, y) if _symbolic(x, y) else np.array([x, y], dtype=float).ravel()


#####################
### Free flight   ###
#####################
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


#################
### ZOH map   ###
#################
def zoh_coefficients(dt, gamma, dt_max=None):
    """
    (e1, a, b) of the exact ZOH map p+ = p + a v + b u, v+ = e1 v + a u (Section 2).

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


def zoh_interval(s, dt, N):
    """Index of the ZOH interval holding time s (since node 0) on a grid of N intervals of
    length dt; times before the grid get the first interval, at or after its end the last."""
    return np.clip(np.floor(s / dt).astype(int), 0, N - 1)


def discrete_matrices(dt, gamma, axes=2):
    """Numeric A_d, B_d of the robot double integrator with viscous friction, state (p, v).
    axes = 2 is the planner's (x, y); the tracker adds the heading (axes = 3)."""
    e1, a, b = zoh_coefficients(dt, gamma)
    I = np.eye(axes)
    A = np.block([[I, a * I], [np.zeros((axes, axes)), e1 * I]])
    B = np.vstack([b * I, a * I])
    return A, B


##############
### Impact ###
##############
def impact(v_R, v_p, n, kap, mass_ratio):
    """
    Robot-puck impact along the contact normal n; returns (v_n, v_p_plus, v_R_plus).

        v_n  = (v_R - v_p) . n
        v_p+ = v_p + kappa v_n n
        v_R+ = v_R - kappa (m_p / m_R) v_n n,   kappa = (1 + e) m_R / (m_R + m_p)

    mass_ratio = m_p / m_R; kappa is ShotParams.kappa.
    """
    v_n = (v_R[0] - v_p[0]) * n[0] + (v_R[1] - v_p[1]) * n[1]
    v_p_plus = _vec2(v_p[0] + kap * v_n * n[0], v_p[1] + kap * v_n * n[1])
    v_R_plus = _vec2(v_R[0] - kap * mass_ratio * v_n * n[0], v_R[1] - kap * mass_ratio * v_n * n[1])
    return v_n, v_p_plus, v_R_plus


def invert(sigma, d, v_p_minus, kap):
    """
    Contact normal and closing speed that send the puck along unit d at speed sigma.

    Requiring v_p+ = sigma d gives kappa v_n n = sigma d - v_p-.
    Returns (n, v_n, phi) with v_n > 0 by construction.
    """
    diff = sigma * np.asarray(d, dtype=float) - np.asarray(v_p_minus, dtype=float)
    norm = np.linalg.norm(diff)
    n = diff / norm
    return n, norm / kap, float(np.arctan2(n[1], n[0]))


########################
### Puck prediction  ###
########################
@dataclass
class PuckFlight:
    """Free flight from the puck state (q, v) at time 0, valid until the first wall hit tau1.
    The planner only strikes before tau1."""

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


##########################
### Drawn puck paths   ###
##########################
def bounce_path(q0, v0, P, t_end, max_hits=30):
    """
    Vertices of the free puck path from now to t_end, for display and tests: the start,
    every wall hit, and the position at t_end. Chains predict() through pure wall
    reflections (exact for this simulator): a side wall flips v_y, an end wall v_x;
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


def fold_y(y_unfolded, w):
    """Map an unfolded y coordinate back onto the real table (triangle wave, period 4w)."""
    return w - np.abs(np.mod(y_unfolded + w, 4 * w) - 2 * w)


def shot_polyline(p_c, v, P):
    """
    Exact vertices of the post-impact puck path: impact point, every side-wall bounce,
    and the end on the goal line (or where drag stops the puck first). Side-wall bounces
    never change v_x and drag only slows the puck, so the path is the straight unfolded
    line folded back onto the table; sampling it in time instead would cut the corners
    at the walls.
    """
    x_c, y_c = p_c
    if v[0] <= 0:
        return np.array([p_c], dtype=float)
    x_end = P.x_g
    if P.gamma >= GAMMA_EPS:
        x_end = min(x_end, x_c + v[0] / P.gamma)  # total x distance before the puck stops
    y_end = y_c + (x_end - x_c) * v[1] / v[0]  # unfolded

    # The unfolded line crosses a wall image wherever y is an odd multiple of w
    lo, hi = sorted((y_c, y_end))
    m = np.arange(np.ceil((lo / P.w - 1) / 2), np.floor((hi / P.w - 1) / 2) + 1)
    y_cross = (2 * m + 1) * P.w
    y_cross = y_cross[(y_cross > lo) & (y_cross < hi)]
    y_cross = y_cross[np.argsort(np.abs(y_cross - y_c))]
    x_cross = x_c + (y_cross - y_c) / (y_end - y_c) * (x_end - x_c) if len(y_cross) else y_cross

    xs = np.concatenate([[x_c], x_cross, [x_end]])
    ys = fold_y(np.concatenate([[y_c], y_cross, [y_end]]), P.w)
    return np.stack([xs, ys], axis=-1)
