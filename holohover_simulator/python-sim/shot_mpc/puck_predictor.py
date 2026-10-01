"""
Closed-form puck prediction with linear drag and wall-hit events (Sections 2 and 7).

All quantities are in the planner frame. Between two events the puck is in free
flight:
    p(t) = q_m + v_m * phi(t - tau_m),   v(t) = v_m * exp(-gamma (t - tau_m))
with phi(t) = (1 - exp(-gamma t)) / gamma (-> t for gamma -> 0).
The wall model is pure reflection, which is exact for this simulator.

Hitting a wall only flips one velocity component: v_y for a side wall, v_x for an end wall. 
So the whole trajectory is fully described by the state just after each hit.

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
class PuckEvents:
    """Event list (tau_m, q_m, v_m): state just after the m-th wall hit, tau_0 = 0.
    Index m is the event number. Event 0 is the current measured state, 
    and event m is the state just after the m-th wall hit. The time between event m and event m+1 is "window m"."""

    tau: np.ndarray  # (M+1,) event times, tau[0] = 0
    q: np.ndarray  # (M+1, 2) position just after the hit
    v: np.ndarray  # (M+1, 2) velocity just after the hit (so the velocity is already reflected)
    walls: list  # wall hit at each event ('start', 'L', 'R', 'near', 'far')
    end: str  # 'goal_for', 'goal_against', 'horizon' or 'stopped'
    t_end: float  # time where the prediction stops
    gamma: float

    @property
    def n_windows(self):
        """Number of windows in the prediction."""
        return len(self.tau)

    def window_end(self, m):
        """Time when window m ends. or t_end if m is the last window."""
        return self.tau[m + 1] if m + 1 < len(self.tau) else self.t_end

    def window_of(self, t):
        """Index of the window containing time t."""
        return int(np.clip(np.searchsorted(self.tau, t, side="right") - 1, 0, len(self.tau) - 1))

    def state_at(self, t, m=None):
        """Position and velocity at time t (in window m, or the window containing t)."""
        if m is None:
            m = self.window_of(t)
        dt = t - self.tau[m]
        return self.q[m] + self.v[m] * flight_phi(dt, self.gamma), self.v[m] * flight_decay(dt, self.gamma)

    def window_data(self, m):
        """(qx, qy, vx, vy, tau_start) of window m, the NLP parameter layout."""
        return np.array([self.q[m, 0], self.q[m, 1], self.v[m, 0], self.v[m, 1], self.tau[m]])

    def sample(self, ts):
        return np.array([self.state_at(t)[0] for t in ts])

    def polyline(self, t):
        """Exact path vertices from time 0 to t: the start, every wall hit before t, and p(t)."""
        hits = [self.q[m] for m in range(1, len(self.tau)) if self.tau[m] < t]
        return np.array([self.q[0], *hits, self.state_at(t)[0]])


def predict_events(q0, v0, P, t_max, max_events=30, v_stop=1e-3):
    """
    Forward-simulate the autonomous puck with pure wall reflection.

    Side walls at y = +-w, our end wall at x = r_p, the opponent end wall at x = x_g.
    Crossing an end wall inside the mouth (|y - y_g| <= h, the simulator's scoring
    rule on the puck centre) ends the prediction with a goal.
    """
    gamma = P.gamma
    w = P.w
    # A collision's position correction can leave the puck marginally outside the effective walls
    q = np.array([np.clip(q0[0], P.r_p, P.x_g), np.clip(q0[1], -w, w)], dtype=float)
    v = np.array(v0, dtype=float)
    taus, qs, vs, walls = [0.0], [q.copy()], [v.copy()], ["start"]
    t = 0.0
    end = "horizon"
    t_end = t_max

    for _ in range(max_events):
        if np.hypot(*v) < v_stop:
            end = "stopped"
            break
        candidates = [
            (hit_time(w, q[1], v[1], gamma), "L"),
            (hit_time(-w, q[1], v[1], gamma), "R"),
            (hit_time(P.x_g, q[0], v[0], gamma), "far"),
            (hit_time(P.r_p, q[0], v[0], gamma), "near"),
        ]
        dt_hit, wall = min(candidates, key=lambda c: c[0])
        if not np.isfinite(dt_hit):
            end = "stopped"
            break
        if t + dt_hit > t_max:
            break

        t += dt_hit
        q = q + v * flight_phi(dt_hit, gamma)
        v = v * flight_decay(dt_hit, gamma)
        if wall in ("L", "R"):
            q[1] = w if wall == "L" else -w
            v[1] = -v[1]
        else:
            q[0] = P.x_g if wall == "far" else P.r_p
            if abs(q[1] - P.y_g) <= P.h:
                end = "goal_for" if wall == "far" else "goal_against"
                t_end = t
                break
            v[0] = -v[0]
        taus.append(t)
        qs.append(q.copy())
        vs.append(v.copy())
        walls.append(wall)

    return PuckEvents(
        tau=np.array(taus), q=np.array(qs), v=np.array(vs), walls=walls,
        end=end, t_end=t_end, gamma=gamma,
    )
