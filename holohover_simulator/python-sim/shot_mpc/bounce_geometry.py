"""
Coarse post-impact model: bounce sequences and the virtual goal (Section 4).

A sequence with first wall s (+1 = L, -1 = R) and k alternating side-wall
bounces scores iff the unfolded straight shot hits the mirrored goal at
y_bar = 2 s k w.

Side-wall bounces never change v_x, so the time to the goal line only depends
on the x motion: T = -ln(1 - gamma (x_g - x_c) / v_x) / gamma. This equals the
L_B / |v| form of the formulation, and the reach constraint becomes
gamma (x_g - x_c) < v_x.
"""

import numpy as np

from .puck_predictor import GAMMA_EPS


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


def y_hit(p_c, v, x_g):
    """We want the y coordinate of the unfolded line that hits the goal line at x_g.
    p_c is the contact point, v is the post-impact puck velocity, x_g is the goal line x coordinate."""
    return p_c[1] + (x_g - p_c[0]) * v[1] / v[0]


def time_to_goal(x_c, v_x, x_g, gamma):
    """Calculate the time it takes for the puck to reach the goal line.
    x_c is the initial x position, v_x is the initial x velocity, x_g is the goal line x coordinate, gamma is the drag coefficient.
    Drag model: dv/dt = -γv 
        => v_x(t) = v_x · e^(−γt)
        => x(t) = x_c + (v_x / gamma) (1 - exp(-γt))
    Solve for t when x(t) = x_g:
        1 − e^(−γT) = γ (x_g − x_c) / v_x
        e^(−γT)     = 1 − γ (x_g − x_c) / v_x   ← this is `arg`
        T           = −ln(arg) / γ"""
    if v_x <= 0:
        return np.inf
    if gamma < GAMMA_EPS:
        return (x_g - x_c) / v_x
    arg = 1.0 - gamma * (x_g - x_c) / v_x
    if arg <= 0:
        return np.inf
    return -np.log(arg) / gamma


def bounce_points(p_c, v, s, k, w, x_g, gamma):
    """Real positions and times of the k bounces of the unfolded shot.
    Finds the intersection of the unfolded line with the goal line and the side walls.
    input:
        p_c: contact point (x, y)
        v: post-impact puck velocity (v_x, v_y)
        s: first wall bounce side (+1 = L, -1 = R)
        k: number of bounces
        w: half table width
        x_g: goal line x coordinate
        gamma: drag coefficient
    output:
        points: array of shape (k, 2) with the (x, y) coordinates of the bounces
        times: array of shape (k,) with the time to reach each bounce from the contact point"""
    yh = y_hit(p_c, v, x_g)
    points, times = [], []
    for j in range(1, k + 1): # j is the bounce index, 1-based
        lam = ((2 * j - 1) * s * w - p_c[1]) / (yh - p_c[1])
        x_j = p_c[0] + lam * (x_g - p_c[0])
        y_j = s * (-1) ** (j - 1) * w
        points.append((x_j, y_j))
        times.append(time_to_goal(p_c[0], v[0], x_j, gamma))
    return np.array(points).reshape(-1, 2), np.array(times)


def fold_y(y_unfolded, w):
    """Map an unfolded y coordinate back onto the real table (triangle wave, period 4w)."""
    return w - np.abs(np.mod(y_unfolded + w, 4 * w) - 2 * w)


def shot_polyline(p_c, v, P):
    """
    Exact vertices of the post-impact puck path: impact point, every side-wall bounce,
    and the end on the goal line (or where drag stops the puck first). Drag only slows
    the puck, so between bounces the path is straight; sampling it in time instead
    would cut the corners at the walls.
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
