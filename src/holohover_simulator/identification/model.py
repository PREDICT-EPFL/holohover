"""2D puck model: free flight on the air table and rebound on the walls.

State: (x, y, yaw, vx, vy, w) in the table frame.

Free flight:
    dv/dt = g - c_v v                    g = (g_x, g_y): table tilt (gravity along the surface)
    dw/dt = -c_w w - c_wc sign(w)        spin with viscous and Coulomb drag
Both have closed forms (`flight`). The normal velocity towards a wall is monotone in time, so
the wall contacts are found exactly (`next_impact`) instead of by time stepping.
A Coulomb term on the linear motion is not in the model: on the recorded data it could not be
separated from c_v (see README).

Wall impact, a 2D version of the independent friction-restitution model of
A. Domenech-Carbo, "On the friction/tangential restitution problem: Independent
friction-restitution modeling of sphere rebound with arbitrary spin", Powder Technology (2022),
doi:10.1016/j.powtec.2022.118141. Normal restitution, friction and tangential restitution are
three independent effects:
  - normal:      v_n' = -e_n v_n, normal impulse P_n = m (1 + e_n) |v_n|
  - friction:    a Coulomb impulse P_f <= mu P_n opposes the slip u_t of the contact point and
                 can at most stop it (slider mu in the paper's sketch)
  - restitution: the tangential compliance loaded by P_f gives back P_et = e_t P_f
                 (spring e_t in series with the slider)
The slip of the contact point is u_t = v.t - r w with t = z x n (n: wall normal into the table).
A tangential impulse P changes it by P / m_t, with m_t = m kappa / (1 + kappa) and
kappa = I / (m r^2). So the slip after the impact is
    u_t' = u_t - sign(u_t) (1 + e_t) min(|u_t|, mu (1 + e_n) |v_n| (1 + kappa) / kappa)
which is -e_t u_t when the contact sticks and the Coulomb limit when it slides. The change is
shared between v_t and w: dv_t = kappa/(1+kappa) du, dw = -du / (r (1+kappa)).
The masses cancel, the impact only needs r and kappa.
"""
import numpy as np
from scipy.optimize import brentq

PARAMETERS = ["c_v", "g_x", "g_y", "c_w", "c_wc", "e_n", "mu", "e_t"]
MAX_IMPACTS = 200  # per rollout, guards against chattering in a corner
V_REST = 0.005  # [m/s] slower bounces against the tilt become sliding along the wall


def _phi1(x):
    """(1 - exp(-x)) / x, stable at x = 0."""
    x = np.asarray(x, dtype=float)
    small = np.abs(x) < 1e-4
    xs = np.where(small, 1.0, x)
    return np.where(small, 1.0 - x / 2.0 + x * x / 6.0, -np.expm1(-xs) / xs)


def _phi2(x):
    """(x - 1 + exp(-x)) / x^2, stable at x = 0."""
    x = np.asarray(x, dtype=float)
    small = np.abs(x) < 1e-3
    xs = np.where(small, 1.0, x)
    return np.where(small, 0.5 - x / 6.0 + x * x / 24.0, (xs + np.expm1(-xs)) / (xs * xs))


def stop_time(s0, c, k):
    """Time for ds/dt = -c s - k to bring s0 >= 0 to rest (inf without Coulomb drag)."""
    if s0 <= 0.0:
        return 0.0
    if k <= 0.0:
        return np.inf
    z = c * s0 / k
    return s0 / k * (np.log1p(z) / z if z > 1e-9 else 1.0 - z / 2.0)


def _decay(s0, c, k, t):
    """Speed and travelled distance at times t >= 0 for ds/dt = -c s - k, s(0) = s0 >= 0."""
    t = np.asarray(t, dtype=float)
    tau = np.minimum(t, stop_time(s0, c, k))
    x = c * tau
    s = s0 * np.exp(-x) - k * tau * _phi1(x)
    dist = s0 * tau * _phi1(x) - k * tau * tau * _phi2(x)
    return np.maximum(s, 0.0), dist


def flight(state, t, p):
    """States (len(t), 6) after a free flight of duration t (array, >= 0) from `state`."""
    t = np.atleast_1d(np.asarray(t, dtype=float))
    x, y, yaw, vx, vy, w = state
    ct = p["c_v"] * t
    f1, f2 = t * _phi1(ct), t * t * _phi2(ct)
    e = np.exp(-ct)
    out = np.empty((t.size, 6))
    out[:, 0] = x + vx * f1 + p["g_x"] * f2
    out[:, 1] = y + vy * f1 + p["g_y"] * f2
    out[:, 3] = vx * e + p["g_x"] * f1
    out[:, 4] = vy * e + p["g_y"] * f1
    sw, dw = _decay(abs(w), p["c_w"], p["c_wc"], t)
    sign = np.sign(w)
    out[:, 2] = yaw + sign * dw
    out[:, 5] = sign * sw
    return out


def _first_contact(h0, a, b, c, horizon):
    """First t in (0, horizon] with h(t) = h0 + a t phi1(ct) + b t^2 phi2(ct) = 0, or inf.

    h is the distance to a wall (h0 >= 0), a and b the normal velocity and acceleration.
    dh/dt = a exp(-ct) + b t phi1(ct) changes sign at most once, at t_ext."""
    def h(t):
        return float(h0 + a * t * _phi1(c * t) + b * t * t * _phi2(c * t))

    if a >= 0.0 and b >= 0.0:
        return np.inf
    # dh/dt = 0: exp(-ct) (a - b/c) = -b/c, or t = -a/b for c = 0
    t_ext = np.inf
    if a * b < 0.0:
        if c > 1e-12:
            arg = -b / (c * a - b)
            if 0.0 < arg < 1.0:
                t_ext = -np.log(arg) / c
        else:
            t_ext = -a / b
    lo = 0.0
    for hi in (min(t_ext, horizon), horizon):
        if hi > lo:
            if h(hi) <= 0.0:
                if h(lo) <= 0.0:
                    return lo
                return brentq(h, lo, hi, xtol=1e-14, rtol=1e-15)
            lo = hi
    return np.inf


def next_impact(state, p, walls, horizon, skip=()):
    """(time, wall index) of the next wall contact within horizon, (inf, None) if none.

    A puck in contact and leaving the wall (h0 = 0, a > 0) can still come back with the tilt."""
    pos = np.asarray(state[0:2])
    v = np.asarray(state[3:5])
    g = np.array([p["g_x"], p["g_y"]])
    best = (np.inf, None)
    for i, wall in enumerate(walls):
        if i in skip:
            continue
        n = wall["normal"]
        h0 = max(float(n @ pos) - wall["offset"], 0.0)
        th = _first_contact(h0, float(n @ v), float(n @ g), p["c_v"], horizon)
        if th < best[0]:
            best = (th, i)
    return best


def impact(state, wall, p, radius, kappa):
    """State right after an impact on `wall` (the position does not change)."""
    n = wall["normal"]
    t = np.array([-n[1], n[0]])  # z x n
    v = np.asarray(state[3:5], dtype=float)
    w = float(state[5])
    vn, vt = float(v @ n), float(v @ t)
    if vn >= 0.0:
        return np.array(state, dtype=float)
    ut = vt - radius * w
    friction = min(abs(ut), p["mu"] * (1.0 + p["e_n"]) * -vn * (1.0 + kappa) / kappa)
    du = -np.sign(ut) * (1.0 + p["e_t"]) * friction
    vn_new = -p["e_n"] * vn
    vt_new = vt + kappa / (1.0 + kappa) * du
    w_new = w - du / (radius * (1.0 + kappa))
    out = np.array(state, dtype=float)
    out[3:5] = vn_new * n + vt_new * t
    out[5] = w_new
    return out


def tangential_ratio(psi, e_n, mu, e_t, kappa):
    """u_t'/u_t of the impact law as a function of psi = |u_t| / |v_n| (for the plots)."""
    psi = np.asarray(psi, dtype=float)
    return 1.0 - (1.0 + e_t) * np.minimum(1.0, mu * (1.0 + e_n) * (1.0 + kappa) / (kappa * psi))


def rollout(state0, t_eval, p, walls, radius, kappa, return_impacts=False):
    """Model states at the times t_eval (sorted, >= 0, relative to state0).

    When the tilt pushes the puck into a wall, the bounces get smaller and smaller (infinitely
    many in finite time). Below V_REST the puck slides along the wall instead: normal velocity
    and the normal part of the tilt are removed (the wall friction of the sliding is neglected)."""
    t_eval = np.asarray(t_eval, dtype=float)
    out = np.empty((t_eval.size, 6))
    impacts = []
    tau, state, idx = 0.0, np.array(state0, dtype=float), 0
    t_end = float(t_eval[-1]) if t_eval.size else 0.0
    pf, sliding = p, set()
    for _ in range(MAX_IMPACTS + 1):
        th, iw = next_impact(state, pf, walls, t_end - tau, skip=sliding)
        t_hit = tau + th
        j = int(np.searchsorted(t_eval, t_hit, side="right")) if np.isfinite(t_hit) else t_eval.size
        if j > idx:
            out[idx:j] = flight(state, t_eval[idx:j] - tau, pf)
        idx = j
        if idx >= t_eval.size or iw is None:
            break
        state = impact(flight(state, [th], pf)[0], walls[iw], p, radius, kappa)
        impacts.append((t_hit, iw))
        tau = t_hit
        n = walls[iw]["normal"]
        g = np.array([pf["g_x"], pf["g_y"]])
        if n @ g < 0.0 and n @ state[3:5] < V_REST:
            state[3:5] -= (n @ state[3:5]) * n
            g -= (n @ g) * n
            pf = {**pf, "g_x": float(g[0]), "g_y": float(g[1])}
            sliding.add(iw)
    else:
        out[idx:] = flight(state, t_eval[idx:] - tau, pf)
    return (out, impacts) if return_impacts else out


def make_wall(name, normal, offset):
    normal = np.asarray(normal, dtype=float)
    return {"name": name, "normal": normal / np.linalg.norm(normal), "offset": float(offset)}


def walls_from_yaml(items):
    return [make_wall(w["name"], w["normal"], w["offset"]) for w in items]


def walls_to_yaml(walls):
    return [{"name": w["name"], "normal": [round(float(c), 6) for c in w["normal"]],
             "offset": round(float(w["offset"]), 5)} for w in walls]
