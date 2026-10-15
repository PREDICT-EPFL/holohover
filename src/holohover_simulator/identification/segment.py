"""Automatic segmentation of a recording into free glides, wall impacts and hits.

1. Events are peaks of the smoothed acceleration. Around each one, lines fitted to the position
   before and after give the velocities, the impact time (where the two lines meet) and the
   contact point.
2. An event is a wall impact when the velocity change points along a table axis and reverses
   the normal velocity. The walls are lines fitted to the contact points of their impacts.
   Impacts too far from their wall, and every other event, are "hits" (robot, hand, ...).
   Events without a velocity change are "flight": an acceleration spike from the mocap while
   the puck keeps flying freely.
3. Data is unusable in gaps, where the puck is slow and where the mocap orientation is mostly
   invalid. In the usable parts each wall impact and each hit gets a segment of up to
   impact_window on both sides, and what is left between them becomes glide segments. Hit
   segments are kept for the review but not fitted (the model has no robot contact).
"""
import numpy as np
from scipy.signal import find_peaks

import model

AXES = [("x_min", (1.0, 0.0)), ("x_max", (-1.0, 0.0)), ("y_min", (0.0, 1.0)), ("y_max", (0.0, -1.0))]
LABELS = ("glide", "wall_impact", "hit")
FIT_LABELS = ("glide", "wall_impact")  # the model explains these, hit segments are not fitted
CONTACT_LABEL = {"wall": "wall_impact", "hit": "hit"}


def is_contact(e):
    """Wall impact or hit with velocities before and after: it can get its own segment."""
    return e["kind"] in CONTACT_LABEL and "v_pre" in e


def _line(t, y):
    """Least squares y = a + b t (y may have several columns)."""
    A = np.column_stack([np.ones_like(t), t])
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    return coef[0], coef[1]


def local_fits(rec, tk, t_lo, t_hi, cfg):
    """Velocities before/after an event near tk, the impact time and the contact point.

    t_lo, t_hi limit the windows (previous/next event, chunk ends). None if too little data."""
    s = cfg["segmentation"]
    eg, fw = s["event_gap"], s["fit_window"]
    pre = rec.window(max(tk - eg - fw, t_lo), tk - eg)
    post = rec.window(tk + eg, min(tk + eg + fw, t_hi))
    if pre.size < 6 or post.size < 6:
        return None
    a1, b1 = _line(rec.t[pre] - tk, rec.p[pre])
    a2, b2 = _line(rec.t[post] - tk, rec.p[post])
    da, db = a1 - a2, b1 - b2
    tau = -float(da @ db) / float(db @ db) if db @ db > 1e-12 else 0.0
    tau = float(np.clip(tau, -eg - 0.01, eg + 0.01))
    p1, p2 = a1 + b1 * tau, a2 + b2 * tau
    out = {"t": tk + tau, "contact": (p1 + p2) / 2.0, "miss": float(np.linalg.norm(p1 - p2)),
           "v_pre": b1, "v_post": b2, "w_pre": None, "w_post": None}
    for key, idx in (("w_pre", pre), ("w_post", post)):
        ok = idx[rec.yaw_ok[idx]]
        if ok.size >= 6:
            out[key] = float(_line(rec.t[ok] - tk, rec.yaw_u[ok])[1])
    return out


def detect_events(rec, cfg):
    s = cfg["segmentation"]
    dt = float(np.median(np.diff(rec.t)))
    acc = np.nan_to_num(np.hypot(rec.a[:, 0], rec.a[:, 1]))
    peaks, _ = find_peaks(acc, height=s["acc_threshold"], distance=max(1, int(0.05 / dt)))
    chunk_of = np.zeros(rec.t.size, dtype=int)
    for c, (a, b) in enumerate(rec.chunks):
        chunk_of[a:b] = c
    events = []
    for j, k in enumerate(peaks):
        a, b = rec.chunks[chunk_of[k]]
        t_lo = max(rec.t[a], rec.t[peaks[j - 1]] + s["event_gap"] if j > 0 else -np.inf)
        t_hi = min(rec.t[b - 1], rec.t[peaks[j + 1]] - s["event_gap"] if j + 1 < peaks.size else np.inf)
        fit = local_fits(rec, rec.t[k], t_lo, t_hi, cfg)
        if fit is None:
            events.append({"t": float(rec.t[k]), "kind": "hit", "wall": None})
            continue
        dv = fit["v_post"] - fit["v_pre"]
        # no velocity change: an acceleration spike from the mocap, the puck keeps flying
        fit["kind"], fit["wall"] = ("hit", None) if np.linalg.norm(dv) > s["min_dv"] else ("flight", None)
        if fit["kind"] == "hit":
            name, n = max(AXES, key=lambda ax: float(dv @ np.array(ax[1])))
            n = np.array(n)
            if dv @ n > np.cos(np.radians(35)) * np.linalg.norm(dv) and fit["v_pre"] @ n < 0 < fit["v_post"] @ n:
                fit["kind"], fit["wall"] = "wall", name
        events.append(fit)
    return events


def estimate_walls(rec, events, cfg):
    """Walls fitted to the contact points; impacts too far from their wall become hits.

    A wall needs min_wall_impacts impacts and no usable data beyond it, otherwise its impacts
    become hits (e.g. "impacts" detected where the mocap data is bad)."""
    s = cfg["segmentation"]
    tol = s["wall_tolerance"]
    fast = np.isfinite(rec.v[:, 0]) & (rec.speed >= s["min_speed"])
    walls = []
    for name, n0 in AXES:
        ev = [e for e in events if e["wall"] == name]
        if not ev:
            continue
        if len(ev) < s["min_wall_impacts"]:
            print(f"wall {name}: only {len(ev)} impacts, not used")
            for e in ev:
                e["kind"], e["wall"] = "hit", None
            continue
        n0 = np.array(n0)
        t0 = np.array([-n0[1], n0[0]])
        c = np.array([e["contact"] for e in ev])
        z, sx = c @ n0, c @ t0
        inl = np.abs(z - np.median(z)) < tol
        n = n0
        if inl.sum() >= 3 and np.ptp(sx[inl]) > 0.3:
            for _ in range(2):  # line fit, then refit without the outliers
                slope = np.polyfit(sx[inl], z[inl], 1)[0]
                phi = -np.arctan(slope)
                n = np.cos(phi) * n0 + np.sin(phi) * t0
                d = c[inl] @ n
                inl = np.abs(c @ n - np.median(d)) < tol
        offset = float(np.median(c[inl] @ n)) if inl.any() else float(np.median(z))
        beyond = int(np.sum(rec.p[fast] @ n < offset - tol))
        if beyond:
            print(f"wall {name}: {beyond} samples of moving puck beyond it, not used")
            for e in ev:
                e["kind"], e["wall"] = "hit", None
            continue
        walls.append(model.make_wall(name, n, offset))
        for e, ok in zip(ev, np.abs(c @ n - offset) < tol):
            if not ok:
                e["kind"], e["wall"] = "hit", None
    return walls


def usable_mask(rec, events, cfg):
    """Samples that can be used for the fit."""
    s = cfg["segmentation"]
    fast = rec.speed >= s["min_speed"]
    # the smoothed speed dips at a contact (the filter averages across the velocity change):
    # there the line fits before and after it decide
    half = cfg["preprocessing"]["smoothing_window"]
    for e in events:
        if is_contact(e):
            near = np.abs(rec.t - e["t"]) <= half
            fast[near] = min(np.linalg.norm(e["v_pre"]), np.linalg.norm(e["v_post"])) >= s["min_speed"]
    ok = np.isfinite(rec.v[:, 0]) & fast
    # mostly invalid orientation: the mocap is confused about the puck
    dt = float(np.median(np.diff(rec.t)))
    n = max(1, int(round(0.1 / dt)))
    bad_yaw = np.convolve((~rec.yaw_ok).astype(float), np.ones(n) / n, mode="same")
    ok &= bad_yaw <= s["bad_tilt_fraction"]
    # keep a margin to everything that is not usable, to hits without line fits and to the chunk ends
    bad_t = list(rec.t[~ok]) + [e["t"] for e in events if e["kind"] == "hit" and not is_contact(e)]
    for a, b in rec.chunks:
        bad_t += [rec.t[a], rec.t[b - 1]]
    bad_t = np.sort(np.array(bad_t))
    if bad_t.size:
        i = np.clip(np.searchsorted(bad_t, rec.t), 1, bad_t.size - 1)
        near = np.minimum(np.abs(rec.t - bad_t[i - 1]), np.abs(rec.t - bad_t[i]))
        ok &= near > s["margin"]
    return ok


def _runs(mask):
    """(start, stop) index ranges where mask is True."""
    d = np.diff(np.concatenate([[0], mask.astype(int), [0]]))
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))


def build_segments(rec, events, ok, cfg):
    s = cfg["segmentation"]
    # a run must not cross a gap
    for a, b in rec.chunks:
        ok[a] = ok[b - 1] = False
    contacts = [e for e in events if is_contact(e)]
    segments = []
    for a, b in _runs(ok):
        rs, re = rec.t[a], rec.t[b - 1]
        inside = [e for e in contacts if rs < e["t"] < re]
        blocks = []  # time intervals taken by contact segments or blocked by dropped contacts
        for j, e in enumerate(inside):
            ti = e["t"]
            w = min(s["impact_window"], ti - rs, re - ti)
            if j > 0:
                w = min(w, (ti - inside[j - 1]["t"]) / 2.0)
            if j + 1 < len(inside):
                w = min(w, (inside[j + 1]["t"] - ti) / 2.0)
            if w >= s["min_impact_window"]:
                segments.append({"label": CONTACT_LABEL[e["kind"]], "t0": ti - w, "t1": ti + w})
                blocks.append((ti - w, ti + w))
            else:
                blocks.append((ti - s["margin"], ti + s["margin"]))
        edges = [rs] + [x for blk in blocks for x in blk] + [re]
        for g0, g1 in zip(edges[0::2], edges[1::2]):
            if g1 - g0 >= s["min_glide"]:
                segments.append({"label": "glide", "t0": g0, "t1": g1})
    segments.sort(key=lambda g: g["t0"])
    return segments


def auto_segment(rec, cfg):
    events = detect_events(rec, cfg)
    walls = estimate_walls(rec, events, cfg)
    ok = usable_mask(rec, events, cfg)
    segments = build_segments(rec, events, ok, cfg)
    return walls, events, segments


def to_yaml(rec, walls, events, segments):
    def r(x, n=4):
        return None if x is None else [round(float(c), n) for c in x] if np.ndim(x) else round(float(x), n)

    return {
        "bag": rec.source,
        "t_ref": rec.t_ref,
        "table_pose_in_mocap": [round(c, 5) for c in rec.table],
        "walls": model.walls_to_yaml(walls),
        "events": [{"t": r(e["t"]), "kind": e["kind"], "wall": e["wall"],
                    **({"contact": r(e["contact"]), "v_pre": r(e["v_pre"]), "v_post": r(e["v_post"]),
                        "w_pre": r(e["w_pre"]), "w_post": r(e["w_post"])} if "contact" in e else {})}
                   for e in events],
        "segments": [{"id": i, "label": g["label"], "t0": r(g["t0"]), "t1": r(g["t1"])}
                     for i, g in enumerate(segments)],
    }
