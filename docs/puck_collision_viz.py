# /// script
# requires-python = ">=3.10"
# dependencies = ["imgui-bundle>=1.5"]
# ///
"""Interactive explorer for a hovercraft-puck collision and the resulting puck trace.

Geometry and masses default to holohover_dial/config.yaml.

Both impacts use the friction-restitution law of holohover_simulator/identification/model.py
(normal restitution e, Coulomb friction mu, tangential restitution e_t), which tracks the
puck spin w. With u_t the slip of the contact point, P_n and P_t the normal and tangential
impulses on the puck and M_n, M_t the effective masses at the contact:
    P_n = M_n (1 + e) g_n                                g_n: closing speed
    P_t = sgn(u_t) (1 + e_t) min(M_t |u_t|, mu P_n)
For the wall M_n = m_P and M_t = m_P kappa / (1 + kappa); for the hit both bodies count.
The wall defaults are the identify.py fit; the hit ones are the simulator's unidentified
guesses. e = 1, mu = 0 gives back the elastic impacts of holohover_dial.

Run with:  uv run --script puck_collision_viz.py

Mouse: drag the puck to move it, drag the hovercraft to slide it around the puck
(it always stays in contact), drag either arrow tip to set speed and direction.

The puck is propagated until it reaches a left/right wall, which either scores or
misses. "mirror arena" shows the same run unfolded: instead of bouncing the puck off the
long walls, the arena is mirrored and the puck flies straight through a stack of
mirrored copies. A long-wall bounce reverses v_n scaled by e and, while the contact
slides, takes c |v_n| off the velocity along the wall, c = mu (1 + e_t) (1 + e). The
copy past a wall is therefore stretched by 1 / e across the wall and sheared: a point
at distance d from the wall moves forward (along the puck's travel) by c d / e. With
"shear" off the copies are only mirrored and stretched (the planner's current goal
images), and the unfolded trace kinks at every bounce. Bounce marks, and the mark at the
hovercraft-puck contact: white = sliding
(the unfolding is exact), yellow = sticking, red = sliding against the travel (spin
dominates the slip). Drag the background to pan, scroll to zoom, or hit "frame trace"
to jump to a view holding the whole trajectory.
"""

import math
import time
from dataclasses import dataclass, field

from imgui_bundle import imgui, immapp

# holohover_dial/config.yaml
TABLE_W, TABLE_H, GOAL_W, FRICTION = 2.13, 1.06, 0.34, 0.0
HOVER_R, HOVER_M = 0.07, 0.116
PUCK_R, PUCK_M = 0.035, 0.023

OUTCOMES = ["-", "GOAL left", "GOAL right", "MISSED left", "MISSED right"]


def col32(r, g, b, a=255):
    return (a << 24) | (b << 16) | (g << 8) | r


C_BG = col32(24, 26, 32)
C_TABLE = col32(120, 130, 150)
C_TABLE_DIM = col32(66, 72, 86)
C_GOAL = col32(90, 200, 120)
C_GOAL_DIM = col32(52, 108, 74)
C_MISS = col32(235, 110, 90)
C_PUCK = col32(235, 180, 70)
C_HOVER = col32(90, 160, 245)
C_VEL = col32(240, 240, 240)
C_POST = col32(255, 120, 90)
REGIME_COLORS = {"slide": col32(255, 255, 255), "stick": col32(250, 210, 60), "reversed": col32(240, 70, 70)}


@dataclass
class State:
    # geometry / physics
    table_w: float = TABLE_W
    table_h: float = TABLE_H
    goal_w: float = GOAL_W
    friction: float = FRICTION
    hover_r: float = HOVER_R
    hover_m: float = HOVER_M
    puck_r: float = PUCK_R
    puck_m: float = PUCK_M
    puck_kappa: float = 0.5  # I / (m r^2), uniform disk
    hover_kappa: float = 0.5
    spin_drag: float = 0.0  # 1/s, viscous decay of the puck spin
    # wall impact: identify.py fit of puck_id_2026-10-13_18-15-39
    wall_e: float = 0.884
    wall_mu: float = 0.070
    wall_et: float = 0.458
    # hit impact: holohover_simulator/config/friction_restitution.yaml, not identified
    hit_e: float = 0.85
    hit_mu: float = 0.20
    hit_et: float = 0.30
    # scene
    puck_x: float = 0.7
    puck_y: float = 0.86
    puck_speed: float = 0.35
    puck_angle: float = -155.0  # degrees
    puck_spin: float = 0.0  # rad/s, counter-clockwise
    hover_speed: float = 1.2
    hover_angle: float = -4.5  # degrees, direction of the hovercraft velocity
    hover_spin: float = 0.0  # rad/s
    contact_angle: float = -120.0  # degrees, where the hovercraft sits around the puck
    # trace
    horizon: float = 5.0
    dt: float = 0.002
    # display
    vec_scale: float = 0.12  # meters of arrow per (m/s)
    mirror_view: bool = False
    shear: bool = True
    zoom: float = 1.0  # multiplier on the scale that fits one arena
    pan_x: float = 0.5 * TABLE_W  # world point at the centre of the view
    pan_y: float = 0.5 * TABLE_H
    frame_request: bool = False
    hide_stale: bool = False
    show_bounces: bool = True
    debounce: float = 0.05
    # runtime
    drag: str = ""
    grab: tuple = (0.0, 0.0)  # world point under the cursor when a drag started
    dirty: bool = True
    changed_at: float = 0.0
    last_sig: tuple = ()
    compute_ms: float = 0.0
    verts: list = field(default_factory=list)  # folded trace, (x, y, t)
    unfolded: list = field(default_factory=list)  # same run in the mirrored arena
    regimes: list = field(default_factory=list)  # wall impact regime of every bounce
    outcome: int = 0
    n_bounces: int = 0
    v_post: tuple = (0.0, 0.0)
    w_post: float = 0.0
    v_hover_post: tuple = (0.0, 0.0)
    w_hover_post: float = 0.0
    collides: bool = True
    hit_regime: str = "slide"


S = State()


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def hover_pos(s):
    d = s.hover_r + s.puck_r
    a = math.radians(s.contact_angle)
    return s.puck_x + d * math.cos(a), s.puck_y + d * math.sin(a)


def impulses(g_n, u_t, inv_mn, inv_mt, e, mu, et):
    """Impulses (P_n, P_t) on the puck and the regime, for the closing speed g_n > 0, the slip u_t of
    the other body's contact point relative to the puck's and the inverse effective masses."""
    p_n = (1.0 + e) * g_n / inv_mn
    stop = abs(u_t) / inv_mt  # impulse that stops the slip
    regime = "stick" if stop <= mu * p_n else "slide"
    return p_n, math.copysign((1.0 + et) * min(stop, mu * p_n), u_t), regime


def post_collision(s):
    """Hit of the hovercraft on the puck, along the line of centres n (hovercraft to puck).

    Returns the velocities and spins after the hit (hovercraft, then puck) and whether the
    bodies are actually closing the gap: with the two already in contact, an impact only
    happens if the relative velocity points along n, otherwise they separate and nothing
    is transferred.
    """
    a = math.radians(s.contact_angle)
    nx, ny = -math.cos(a), -math.sin(a)
    tx, ty = -ny, nx  # z x n
    v1x = s.hover_speed * math.cos(math.radians(s.hover_angle))
    v1y = s.hover_speed * math.sin(math.radians(s.hover_angle))
    v2x = s.puck_speed * math.cos(math.radians(s.puck_angle))
    v2y = s.puck_speed * math.sin(math.radians(s.puck_angle))
    dvx, dvy = v1x - v2x, v1y - v2y
    g_n = dvx * nx + dvy * ny
    if g_n < 1e-9:
        return (v1x, v1y), s.hover_spin, (v2x, v2y), s.puck_spin, False, "slide"
    u_t = dvx * tx + dvy * ty + s.hover_r * s.hover_spin + s.puck_r * s.puck_spin
    inv_mn = 1.0 / s.puck_m + 1.0 / s.hover_m
    inv_mt = inv_mn + 1.0 / (s.puck_kappa * s.puck_m) + 1.0 / (s.hover_kappa * s.hover_m)
    p_n, p_t, regime = impulses(g_n, u_t, inv_mn, inv_mt, s.hit_e, s.hit_mu, s.hit_et)
    jx, jy = p_n * nx + p_t * tx, p_n * ny + p_t * ty  # impulse on the puck
    return (
        (v1x - jx / s.hover_m, v1y - jy / s.hover_m),
        s.hover_spin - p_t / (s.hover_kappa * s.hover_m * s.hover_r),
        (v2x + jx / s.puck_m, v2y + jy / s.puck_m),
        s.puck_spin - p_t / (s.puck_kappa * s.puck_m * s.puck_r),
        True,
        regime,
    )


def wall_impact(s, vx, vy, w, ny):
    """Bounce on a long wall with normal (0, ny) into the table: (vx, vy, w) after it and the regime.

    The impulses are per unit puck mass (M_n = m_P, M_t = m_P kappa / (1 + kappa)). "reversed":
    sliding with the slip against the travel along the wall, so the spin dominates it."""
    k = s.puck_kappa
    v_t = -ny * vx  # t = z x n = (-ny, 0)
    slip = v_t - s.puck_r * w  # puck contact point along t; the wall's is at rest
    p_n, p_t, regime = impulses(-ny * vy, -slip, 1.0, 1.0 + 1.0 / k, s.wall_e, s.wall_mu, s.wall_et)
    if regime == "slide" and slip * v_t < 0.0:
        regime = "reversed"
    return vx - ny * p_t, vy + ny * p_n, w - p_t / (k * s.puck_r), regime


IDENTITY = ((1.0, 0.0, 0.0, 1.0), (0.0, 0.0))  # affine map: (a, b, c, d) and offset (ox, oy)


def affine_apply(T, x, y):
    (a, b, c, d), (ox, oy) = T
    return a * x + b * y + ox, c * x + d * y + oy


def affine_compose(T, R):
    """T after R."""
    (a, b, c, d), _ = T
    (p, q, r, u), (rx, ry) = R
    return (a * p + b * r, a * q + b * u, c * p + d * r, c * q + d * u), affine_apply(T, rx, ry)


def wall_unfold(y_w, nu, sigma, e, c):
    """Unfolding past the long wall y = y_w (nu = +1: top wall, the puck comes back below it;
    -1: bottom) for a puck travelling along sigma = sgn(v_x): a point at distance d from the
    wall goes to its mirror image stretched by 1 / e and is pushed forward by c d / e."""
    k = sigma * nu * c / e
    return (1.0, -k, 0.0, -1.0 / e), (k * y_w, y_w * (1.0 + 1.0 / e))


def unfold_params(s):
    """(e, c) of the unfolding: c = 0 without the shear."""
    return s.wall_e, s.wall_mu * (1.0 + s.wall_et) * (1.0 + s.wall_e) if s.shear else 0.0


def simulate(s, vx, vy, w):
    """Propagate the puck up to the first left/right wall.

    Returns the folded polyline, the same polyline unfolded (long-wall bounces
    replaced by the mirrored, stretched and sheared copies of the arena), the regime
    of every bounce and the outcome index into OUTCOMES.
    """
    r, W, h, dt = s.puck_r, s.table_w, s.table_h, s.dt
    gy0 = 0.5 * h - 0.5 * s.goal_w
    gy1 = 0.5 * h + 0.5 * s.goal_w
    damp = 1.0 - s.friction * dt
    spin_damp = 1.0 - s.spin_drag * dt
    e, c = unfold_params(s)
    x, y, t = s.puck_x, s.puck_y, 0.0
    T = IDENTITY  # unfolded point = T(folded point)
    verts = [(x, y, 0.0)]
    unfolded = [(x, y, 0.0)]
    regimes = []

    for _ in range(max(1, int(round(s.horizon / dt)))):
        x0, y0 = x, y
        x += vx * dt
        y += vy * dt
        vx *= damp
        vy *= damp
        w *= spin_damp
        t += dt

        wall = r if x - r < 0.0 else (W - r if x + r > W else None)
        if wall is not None:
            f = clamp((wall - x0) / (x - x0), 0.0, 1.0) if abs(x - x0) > 1e-12 else 0.0
            xh, yh, th = wall, y0 + f * (y - y0), t - (1.0 - f) * dt
            verts.append((xh, yh, th))
            unfolded.append((*affine_apply(T, xh, yh), th))
            left = wall == r
            in_goal = s.goal_w > 0.0 and gy0 <= yh <= gy1
            outcome = (1 if left else 2) if in_goal else (3 if left else 4)
            return verts, unfolded, regimes, outcome

        for y_w, ny in ((r, 1.0), (h - r, -1.0)):
            if (y - y_w) * ny < 0.0:
                f = (y_w - y0) / (y - y0)
                xc, tc = x0 + f * (x - x0), t - (1.0 - f) * dt
                sigma = 1.0 if vx >= 0.0 else -1.0
                vx, vy, w, regime = wall_impact(s, vx, vy, w, ny)
                x, y = xc + vx * (1.0 - f) * dt, y_w + vy * (1.0 - f) * dt
                verts.append((xc, y_w, tc))
                unfolded.append((*affine_apply(T, xc, y_w), tc))
                T = affine_compose(T, wall_unfold(y_w, -ny, sigma, e, c))
                regimes.append(regime)

    verts.append((x, y, t))
    unfolded.append((*affine_apply(T, x, y), t))
    return verts, unfolded, regimes, 0


def signature(s):
    return (
        s.table_w, s.table_h, s.goal_w, s.friction,
        s.hover_r, s.hover_m, s.puck_r, s.puck_m,
        s.puck_kappa, s.hover_kappa, s.spin_drag,
        s.wall_e, s.wall_mu, s.wall_et, s.hit_e, s.hit_mu, s.hit_et,
        s.puck_x, s.puck_y, s.puck_speed, s.puck_angle, s.puck_spin,
        s.hover_speed, s.hover_angle, s.hover_spin, s.contact_angle,
        s.horizon, s.dt, s.shear,
    )


def recompute(s):
    t0 = time.perf_counter()
    s.v_hover_post, s.w_hover_post, s.v_post, s.w_post, s.collides, s.hit_regime = post_collision(s)
    if s.collides:
        s.verts, s.unfolded, s.regimes, s.outcome = simulate(s, *s.v_post, s.w_post)
    else:
        s.verts, s.unfolded, s.regimes, s.outcome = [], [], [], 0
    s.n_bounces = len(s.regimes)
    s.compute_ms = 1e3 * (time.perf_counter() - t0)


def draw_arrow(dl, p0, p1, col, thickness=2.0):
    dx, dy = p1.x - p0.x, p1.y - p0.y
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return
    dl.add_line(p0, p1, col, thickness)
    ux, uy = dx / length, dy / length
    head = min(13.0, 0.45 * length)
    bx, by = p1.x - ux * head, p1.y - uy * head
    dl.add_triangle_filled(
        p1,
        imgui.ImVec2(bx - uy * 0.4 * head, by + ux * 0.4 * head),
        imgui.ImVec2(bx + uy * 0.4 * head, by - ux * 0.4 * head),
        col,
    )


def frame_trace(s, avail, margin, fit):
    """Set pan and zoom so the whole trajectory (folded or unfolded) is visible."""
    trace = s.unfolded if s.mirror_view else s.verts
    xs = [p[0] for p in trace] or [0.0, s.table_w]
    ys = [p[1] for p in trace] or [0.0, s.table_h]
    xlo, xhi = min(xs + [0.0]), max(xs + [s.table_w])
    ylo, yhi = min(ys + [0.0]), max(ys + [s.table_h])
    span_x, span_y = max(xhi - xlo, 1e-3), max(yhi - ylo, 1e-3)
    s.zoom = clamp(min((avail.x - 2 * margin) / span_x,
                       (avail.y - 2 * margin) / span_y) / fit, 0.02, 20.0)
    s.pan_x, s.pan_y = 0.5 * (xlo + xhi), 0.5 * (ylo + yhi)


def draw_table(dl, w2s, s, T, dim):
    """One copy of the arena, the image of the table under the affine map T."""
    c_wall = C_TABLE_DIM if dim else C_TABLE
    c_goal = C_GOAL_DIM if dim else C_GOAL

    def line(x0, y0, x1, y1, col, thickness):
        dl.add_line(w2s(*affine_apply(T, x0, y0)), w2s(*affine_apply(T, x1, y1)), col, thickness)

    gy0 = 0.5 * s.table_h - 0.5 * s.goal_w
    gy1 = 0.5 * s.table_h + 0.5 * s.goal_w
    for x in (0.0, s.table_w):
        line(x, 0.0, x, gy0, c_wall, 2.0)
        line(x, gy1, x, s.table_h, c_wall, 2.0)
        line(x, gy0, x, gy1, c_goal, 4.0)
    line(0.0, 0.0, s.table_w, 0.0, c_wall, 2.0)
    line(0.0, s.table_h, s.table_w, s.table_h, c_wall, 2.0)
    line(0.5 * s.table_w, 0.0, 0.5 * s.table_w, s.table_h, col32(70, 76, 90), 1.0)


def draw_arena(s):
    avail = imgui.get_content_region_avail()
    if avail.x < 50.0 or avail.y < 50.0:
        return
    origin = imgui.get_cursor_screen_pos()
    imgui.invisible_button("arena", avail)
    dl = imgui.get_window_draw_list()
    dl.add_rect_filled(origin, imgui.ImVec2(origin.x + avail.x, origin.y + avail.y), C_BG)

    margin = 40.0
    fit = min((avail.x - 2 * margin) / s.table_w, (avail.y - 2 * margin) / s.table_h)
    cx, cy = origin.x + 0.5 * avail.x, origin.y + 0.5 * avail.y
    mouse = imgui.get_io().mouse_pos

    if s.frame_request:
        s.frame_request = False
        frame_trace(s, avail, margin, fit)
    scale = fit * s.zoom

    wheel = imgui.get_io().mouse_wheel
    if wheel != 0.0 and imgui.is_item_hovered():
        gx = s.pan_x + (mouse.x - cx) / scale
        gy = s.pan_y - (mouse.y - cy) / scale
        s.zoom = clamp(s.zoom * (1.12 ** wheel), 0.02, 20.0)
        scale = fit * s.zoom
        s.pan_x = gx - (mouse.x - cx) / scale
        s.pan_y = gy + (mouse.y - cy) / scale

    def w2s(x, yu):
        return imgui.ImVec2(cx + (x - s.pan_x) * scale, cy - (yu - s.pan_y) * scale)

    def s2w(p):
        return s.pan_x + (p.x - cx) / scale, s.pan_y - (p.y - cy) / scale

    def handles(hx, hy):
        return (
            w2s(s.puck_x + s.vec_scale * s.puck_speed * math.cos(math.radians(s.puck_angle)),
                s.puck_y + s.vec_scale * s.puck_speed * math.sin(math.radians(s.puck_angle))),
            w2s(hx + s.vec_scale * s.hover_speed * math.cos(math.radians(s.hover_angle)),
                hy + s.vec_scale * s.hover_speed * math.sin(math.radians(s.hover_angle))),
        )

    hx, hy = hover_pos(s)
    p_tip, h_tip = handles(hx, hy)

    # --- mouse interaction: grab a handle, otherwise pan the view ----------
    if imgui.is_item_clicked(0):
        s.drag = "pan"
        best = 22.0
        for name, p in (("puck_vel", p_tip), ("hover_vel", h_tip),
                        ("hover", w2s(hx, hy)), ("puck", w2s(s.puck_x, s.puck_y))):
            d = math.hypot(mouse.x - p.x, mouse.y - p.y)
            if d < best:
                best, s.drag = d, name
        s.grab = s2w(mouse)
    if not imgui.is_item_active():
        s.drag = ""
    if s.drag:
        mx, my = s2w(mouse)
        if s.drag == "pan":
            s.pan_x = s.grab[0] - (mouse.x - cx) / scale
            s.pan_y = s.grab[1] + (mouse.y - cy) / scale
        elif s.drag == "puck":
            s.puck_x = clamp(mx, s.puck_r, s.table_w - s.puck_r)
            s.puck_y = clamp(my, s.puck_r, s.table_h - s.puck_r)
        elif s.drag == "hover":
            s.contact_angle = math.degrees(math.atan2(my - s.puck_y, mx - s.puck_x))
        elif s.drag == "puck_vel":
            dx, dy = mx - s.puck_x, my - s.puck_y
            if math.hypot(dx, dy) > 1e-4:
                s.puck_angle = math.degrees(math.atan2(dy, dx))
                s.puck_speed = clamp(math.hypot(dx, dy) / s.vec_scale, 0.0, 5.0)
        elif s.drag == "hover_vel":
            dx, dy = mx - hx, my - hy
            if math.hypot(dx, dy) > 1e-4:
                s.hover_angle = math.degrees(math.atan2(dy, dx))
                s.hover_speed = clamp(math.hypot(dx, dy) / s.vec_scale, 0.0, 5.0)
        hx, hy = hover_pos(s)
        p_tip, h_tip = handles(hx, hy)

    # --- arena: the copies past the long walls, sheared along the puck's travel
    if s.mirror_view:
        e, c = unfold_params(s)
        sigma = 1.0 if s.v_post[0] >= 0.0 else -1.0
        top = wall_unfold(s.table_h - s.puck_r, 1.0, sigma, e, c)
        bottom = wall_unfold(s.puck_r, -1.0, sigma, e, c)
        for first, second in ((top, bottom), (bottom, top)):
            T = IDENTITY
            for j in range(2):
                T = affine_compose(T, first if j % 2 == 0 else second)
                draw_table(dl, w2s, s, T, True)
    draw_table(dl, w2s, s, IDENTITY, False)

    # --- trace -------------------------------------------------------------
    trace = s.unfolded if s.mirror_view else s.verts
    if not (s.dirty and s.hide_stale) and len(trace) > 1:
        alpha = 110 if s.dirty else 255
        total = max(trace[-1][2], 1e-6)
        for (x0, y0, t0), (x1, y1, _) in zip(trace, trace[1:]):
            f = 1.0 - 0.65 * (t0 / total)
            dl.add_line(w2s(x0, y0), w2s(x1, y1),
                        col32(int(90 * f), int(225 * f), int(235 * f), alpha), 2.0)
        if s.show_bounces:
            for (x, y, _), regime in zip(trace[1:-1], s.regimes):
                dl.add_circle_filled(w2s(x, y), 4.5, REGIME_COLORS[regime] & 0x00FFFFFF | alpha << 24)
        ex, ey, et = trace[-1]
        end = w2s(ex, ey)
        dl.add_circle(end, s.puck_r * scale, col32(90, 225, 235, alpha), 0, 1.5)
        if s.outcome:
            scored = s.outcome in (1, 2)
            col = C_GOAL if scored else C_MISS
            dl.add_circle_filled(end, 7.0, col)
            label = f"{OUTCOMES[s.outcome]} @ {et:.2f}s"
            lx = end.x + 12.0
            if lx + imgui.calc_text_size(label).x > origin.x + avail.x - 4.0:
                lx = end.x - 12.0 - imgui.calc_text_size(label).x
            dl.add_text(imgui.ImVec2(lx, end.y - 7.0), col, label)

    # --- bodies ------------------------------------------------------------
    dl.add_circle_filled(w2s(hx, hy), s.hover_r * scale, col32(90, 160, 245, 70))
    dl.add_circle(w2s(hx, hy), s.hover_r * scale, C_HOVER, 0, 2.0)
    dl.add_circle_filled(w2s(s.puck_x, s.puck_y), s.puck_r * scale, col32(235, 180, 70, 90))
    dl.add_circle(w2s(s.puck_x, s.puck_y), s.puck_r * scale, C_PUCK, 0, 2.0)

    draw_arrow(dl, w2s(hx, hy), h_tip, C_VEL)
    draw_arrow(dl, w2s(s.puck_x, s.puck_y), p_tip, C_VEL)
    if s.collides:
        draw_arrow(dl, w2s(s.puck_x, s.puck_y),
                   w2s(s.puck_x + s.vec_scale * s.v_post[0], s.puck_y + s.vec_scale * s.v_post[1]),
                   C_POST, 2.5)
        draw_arrow(dl, w2s(hx, hy),
                   w2s(hx + s.vec_scale * s.v_hover_post[0], hy + s.vec_scale * s.v_hover_post[1]),
                   col32(255, 120, 90, 140), 2.0)
        a = math.radians(s.contact_angle)
        dl.add_circle_filled(w2s(s.puck_x + s.puck_r * math.cos(a), s.puck_y + s.puck_r * math.sin(a)),
                             4.5, REGIME_COLORS[s.hit_regime])
    else:
        p = w2s(s.puck_x, s.puck_y)
        dl.add_text(imgui.ImVec2(p.x + 12.0, p.y + 12.0), C_MISS, "no impact: bodies separating")

    hint = "white = pre-collision velocity   orange = post-collision   cyan = puck trace"
    if s.mirror_view:
        hint += ("   |   unfolded: mirrored, stretched and sheared copies, straight while every bounce slides"
                 if s.shear else "   |   unfolded: mirrored and stretched copies only")
    dl.add_text(imgui.ImVec2(origin.x + 8.0, origin.y + 8.0), col32(150, 155, 170), hint)


def draw_controls(s):
    imgui.text("Puck trace after collision")
    imgui.separator()
    imgui.push_item_width(150.0)

    _, s.horizon = imgui.slider_float("horizon [s]", s.horizon, 0.5, 20.0)
    _, s.dt = imgui.slider_float("sim dt [s]", s.dt, 0.0005, 0.01)

    imgui.text("Velocities")
    _, s.hover_speed = imgui.slider_float("hover |v| [m/s]", s.hover_speed, 0.0, 5.0)
    _, s.hover_angle = imgui.slider_float("hover angle [deg]", s.hover_angle, -180.0, 180.0)
    _, s.puck_speed = imgui.slider_float("puck |v| [m/s]", s.puck_speed, 0.0, 5.0)
    _, s.puck_angle = imgui.slider_float("puck angle [deg]", s.puck_angle, -180.0, 180.0)
    _, s.contact_angle = imgui.slider_float("contact angle [deg]", s.contact_angle, -180.0, 180.0)
    _, s.hover_spin = imgui.slider_float("hover spin [rad/s]", s.hover_spin, -20.0, 20.0)
    _, s.puck_spin = imgui.slider_float("puck spin [rad/s]", s.puck_spin, -100.0, 100.0)

    imgui.text("Position")
    _, s.puck_x = imgui.slider_float("puck x [m]", s.puck_x, s.puck_r, s.table_w - s.puck_r)
    _, s.puck_y = imgui.slider_float("puck y [m]", s.puck_y, s.puck_r, s.table_h - s.puck_r)

    imgui.text("Physics (config.yaml)")
    _, s.hover_m = imgui.slider_float("hover mass [kg]", s.hover_m, 0.05, 0.30)
    _, s.puck_m = imgui.slider_float("puck mass [kg]", s.puck_m, 0.005, 0.10)
    _, s.hover_r = imgui.slider_float("hover radius [m]", s.hover_r, 0.02, 0.15)
    _, s.puck_r = imgui.slider_float("puck radius [m]", s.puck_r, 0.01, 0.10)
    _, s.friction = imgui.slider_float("friction [1/s]", s.friction, 0.0, 1.0)
    _, s.table_w = imgui.slider_float("table w [m]", s.table_w, 1.0, 3.0)
    _, s.table_h = imgui.slider_float("table h [m]", s.table_h, 0.5, 2.0)
    _, s.goal_w = imgui.slider_float("goal width [m]", s.goal_w, 0.0, 1.0)
    _, s.puck_kappa = imgui.slider_float("puck I/(m r^2)", s.puck_kappa, 0.1, 1.0)
    _, s.hover_kappa = imgui.slider_float("hover I/(m r^2)", s.hover_kappa, 0.1, 1.0)
    _, s.spin_drag = imgui.slider_float("spin drag [1/s]", s.spin_drag, 0.0, 2.0)

    imgui.text("Wall impact (identified)")
    _, s.wall_e = imgui.slider_float("wall e_n", s.wall_e, 0.3, 1.0)
    _, s.wall_mu = imgui.slider_float("wall mu", s.wall_mu, 0.0, 0.5)
    _, s.wall_et = imgui.slider_float("wall e_t", s.wall_et, -1.0, 1.0)
    imgui.text("Hit impact (guess)")
    _, s.hit_e = imgui.slider_float("hit e_n", s.hit_e, 0.0, 1.0)
    _, s.hit_mu = imgui.slider_float("hit mu", s.hit_mu, 0.0, 0.5)
    _, s.hit_et = imgui.slider_float("hit e_t", s.hit_et, -1.0, 1.0)

    imgui.text("Display")
    _, s.vec_scale = imgui.slider_float("arrow scale [m per m/s]", s.vec_scale, 0.02, 0.5)
    _, s.debounce = imgui.slider_float("debounce [s]", s.debounce, 0.0, 0.5)
    imgui.pop_item_width()
    _, s.mirror_view = imgui.checkbox("mirror arena (unfold bounces)", s.mirror_view)
    _, s.shear = imgui.checkbox("shear the copies (wall friction)", s.shear)
    _, s.show_bounces = imgui.checkbox("mark bounces", s.show_bounces)
    _, s.hide_stale = imgui.checkbox("hide stale trace", s.hide_stale)

    if imgui.button("frame trace"):
        s.frame_request = True
    imgui.same_line()
    if imgui.button("reset view"):
        s.zoom, s.pan_x, s.pan_y = 1.0, 0.5 * s.table_w, 0.5 * s.table_h
    imgui.text_disabled("drag background to pan, scroll to zoom")

    imgui.separator()
    if s.collides:
        imgui.text(f"puck |v| after impact : {math.hypot(*s.v_post):.3f} m/s")
        imgui.text(f"puck spin after impact: {s.w_post:.1f} rad/s")
        imgui.text(f"hit regime            : {s.hit_regime}")
        imgui.text(f"wall bounces          : {s.n_bounces}")
        imgui.text(f"regimes               : {' '.join(s.regimes) or '-'}")
        imgui.text(f"outcome               : {OUTCOMES[s.outcome]}")
    else:
        imgui.text("no impact: the two bodies")
        imgui.text("are separating, no trace")
        imgui.text("")
    k = s.puck_kappa
    imgui.text(f"shear c = mu(1+e_t)(1+e_n): {s.wall_mu * (1 + s.wall_et) * (1 + s.wall_e):.3f}")
    imgui.text(f"slides iff |u_t|/|v_n| > {s.wall_mu * (1 + s.wall_e) * (1 + k) / k:.3f}")
    imgui.text(f"trace compute         : {s.compute_ms:.1f} ms")
    if s.dirty:
        imgui.text("recomputing...")
    if imgui.button("reset"):
        keep = ("horizon", "dt", "vec_scale", "debounce", "mirror_view", "shear",
                "hide_stale", "show_bounces", "zoom", "pan_x", "pan_y")
        values = [getattr(s, f) for f in keep]
        fresh = State()
        for f, v in zip(keep, values):
            setattr(fresh, f, v)
        s.__dict__.update(fresh.__dict__)


def gui():
    now = imgui.get_time()
    sig = signature(S)
    if sig != S.last_sig:
        S.last_sig = sig
        S.dirty = True
        S.changed_at = now
    if S.dirty and now - S.changed_at >= S.debounce:
        recompute(S)
        S.dirty = False

    imgui.begin_child("controls", imgui.ImVec2(330.0, 0.0))
    draw_controls(S)
    imgui.end_child()
    imgui.same_line()
    imgui.begin_child("canvas", imgui.ImVec2(0.0, 0.0))
    draw_arena(S)
    imgui.end_child()


if __name__ == "__main__":
    immapp.run(gui_function=gui, window_title="holohover puck collision", window_size=(1500, 900), fps_idle=0)
