"""Puck-robot impacts: measured from the raw poses of both bodies, and a two-body impact law fitted to them.

    uv run identify.py hits BAG [BAG ...]          measure the hits, fit -> hits.yaml (and plots with --save)

Detection: the robot pose is interpolated at the puck samples; every close approach of the two centres
(distance below puck + robot radius + contact_margin) is a candidate. Around it, lines fitted to the
puck position before and after give its velocities, the impact time (where the two lines meet) and how
well the two lines meet (miss). The robot is still before the impact (line fit) and pulled back by its
controller right after it (quadratic fit, its velocity at the impact time). The yaw rates of both come
from the same windows. A hit is used when every fit is clean (rms below max_fit_rms), the lines meet,
the bodies approach before and separate after, and the centre distance at the impact is the sum of the
radii (within contact_tolerance).

Impact law (model.impact for two moving discs): n from the robot centre to the puck centre,
t = z x n. With the relative velocity v = v_puck - v_robot, the contact point slip is
    u_t = v.t - r_p w_p - r_r w_r,      u_n = v.n < 0
The normal impulse on the puck is P_n = (1 + e_n) |u_n| / (1/m_p + 1/m_r). Tangentially the contact
sticks (u_t' = -e_t u_t) or slides at the Coulomb limit mu P_n, with the tangential effective mass
1/m_t = 1/m_p + 1/m_r + r_p^2/I_p + r_r^2/I_r:
    du_t = -sign(u_t) (1 + e_t) min(|u_t|, mu P_n / m_t),   P_t = m_t du_t
For a fixed wall (m_r, I_r -> inf, r_r w_r = 0) this is the wall law of model.py.

The fit is on the velocities right after the impact (puck v, w and robot v, w) from the measured
velocities before it: e_n, mu, e_t and the puck mass (the robot recoil fixes the mass ratio).
"""
import glob
import math
import os

import numpy as np
import yaml
from scipy.optimize import least_squares

import data

PARAMETERS = ["e_n", "mu", "e_t", "m_p"]


# ---------------------------------------------------------------- impact law

def impact(vp, wp, vr, wr, n, p, rc):
    """Post-impact (vp, wp, vr, wr) of puck and robot; n points from the robot to the puck.

    p: e_n, mu, e_t, m_p; rc: robot and puck constants (r_p, kappa, m_r, I_r, r_r)."""
    n = np.asarray(n, float)
    t = np.array([-n[1], n[0]])
    m_p, m_r = p["m_p"], rc["m_r"]
    I_p = rc["kappa"] * m_p * rc["r_p"] ** 2
    v = np.asarray(vp, float) - np.asarray(vr, float)
    un = float(v @ n)
    if un >= 0.0:
        return np.asarray(vp, float), float(wp), np.asarray(vr, float), float(wr)
    ut = float(v @ t) - rc["r_p"] * wp - rc["r_r"] * wr
    Pn = (1.0 + p["e_n"]) * -un / (1.0 / m_p + 1.0 / m_r)
    m_t = 1.0 / (1.0 / m_p + 1.0 / m_r + rc["r_p"] ** 2 / I_p + rc["r_r"] ** 2 / rc["I_r"])
    du = -np.sign(ut) * (1.0 + p["e_t"]) * min(abs(ut), p["mu"] * Pn / m_t)
    Pt = m_t * du
    P = Pn * n + Pt * t  # impulse on the puck
    return (np.asarray(vp, float) + P / m_p, wp - rc["r_p"] * Pt / I_p,
            np.asarray(vr, float) - P / m_r, wr - rc["r_r"] * Pt / rc["I_r"])


def constants(cfg):
    h = cfg["robot_hits"]["robot"]
    return {"r_p": cfg["puck"]["radius"], "kappa": cfg["puck"]["kappa"],
            "m_r": h["mass"], "I_r": h["inertia"], "r_r": h["radius"]}


# ---------------------------------------------------------------- data

def _read(mcap_file, topics):
    from mcap_ros2.reader import read_ros2_messages
    rows = {k: [] for k in topics}
    for m in read_ros2_messages(mcap_file, topics=list(topics)):
        s = m.ros_msg.header.stamp
        pose = m.ros_msg.pose
        q = pose.orientation
        rows[m.channel.topic].append([s.sec + s.nanosec * 1e-9, pose.position.x, pose.position.y,
                                      data._yaw(q), math.hypot(q.x, q.y)])
    out = {}
    for k, r in rows.items():
        a = np.array(r) if r else np.empty((0, 5))
        a = a[np.argsort(a[:, 0], kind="stable")]
        out[k] = a[np.concatenate([[True], np.diff(a[:, 0]) > 0])] if len(a) else a
    return out


def load(bag, cfg):
    """Puck and robot samples (t, x, y, yaw, tilt), table frame (when the bag has the table pose), cached."""
    mcap_file, out_dir = data.bag_paths(bag)
    topics = cfg["topics"]
    names = {"puck": topics["pose"], "robot": cfg["robot_hits"]["robot_topic"], "table": topics["table"]}
    cache = os.path.join(out_dir, "cache", "robot_hits.npz")
    key = f"1|{mcap_file}|{os.path.getmtime(mcap_file)}|{names}"
    if os.path.exists(cache) and str(np.load(cache)["key"]) == key:
        c = np.load(cache)
        raw = {k: c[k] for k in names}
    else:
        print(f"reading {mcap_file} ...")
        r = _read(mcap_file, set(names.values()))
        raw = {k: r[v] for k, v in names.items()}
        os.makedirs(os.path.dirname(cache), exist_ok=True)
        np.savez(cache, key=key, **raw)
    for k in ("puck", "robot"):
        if not len(raw[k]):
            raise RuntimeError(f"no messages on {names[k]} in {mcap_file}")
    tb = raw["table"]
    if len(tb):
        tx, ty = np.median(tb[:, 1]), np.median(tb[:, 2])
        tyaw = math.atan2(np.median(np.sin(tb[:, 3])), np.median(np.cos(tb[:, 3])))
        c, s = math.cos(tyaw), math.sin(tyaw)
        for k in ("puck", "robot"):
            a = raw[k].copy()
            dx, dy = a[:, 1] - tx, a[:, 2] - ty
            a[:, 1], a[:, 2] = c * dx + s * dy, -s * dx + c * dy
            a[:, 3] = np.angle(np.exp(1j * (a[:, 3] - tyaw)))
            raw[k] = a
    return raw, out_dir


# ---------------------------------------------------------------- measurement

def _poly(t, y, deg):
    """Least squares polynomial (coefficients lowest first) and the rms of the residual."""
    A = np.vander(t, deg + 1, increasing=True)
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    res = y - A @ coef
    return coef, float(np.sqrt(np.mean(res ** 2)))


def _yaw_rate(t, yaw, tilt, t0, deg, cfg):
    """Yaw rate at t0 from the samples with a valid orientation, or None."""
    ok = tilt <= cfg["preprocessing"]["max_tilt"]
    if ok.sum() < cfg["robot_hits"]["min_samples"]:
        return None
    yu = np.unwrap(yaw[ok])
    coef, rms = _poly(t[ok] - t0, yu, deg)
    return float(coef[1]) if rms < 0.05 else None


def measure(raw, cfg):
    """Every candidate contact with its measured velocities and the reason it is rejected (None: used)."""
    h = cfg["robot_hits"]
    rc = constants(cfg)
    P, R = raw["puck"], raw["robot"]
    t = P[:, 0]
    rx = np.interp(t, R[:, 0], R[:, 1])
    ry = np.interp(t, R[:, 0], R[:, 2])
    d = np.hypot(P[:, 1] - rx, P[:, 2] - ry)
    r_sum = rc["r_p"] + rc["r_r"]
    close = np.flatnonzero(d < r_sum + h["contact_margin"])
    if not close.size:
        return []
    groups = np.split(close, np.flatnonzero(np.diff(t[close]) > 0.05) + 1)
    tk_all = [float(t[g[np.argmin(d[g])]]) for g in groups]
    eg, fw, rw = h["event_gap"], h["fit_window"], h["robot_window"]
    t0 = float(t[0])  # hit times are reported from the first puck sample
    hits = []
    for j, tk in enumerate(tk_all):
        hit = {"t": tk - t0}
        hits.append(hit)
        others = [x for i, x in enumerate(tk_all) if i != j and abs(x - tk) < h["quiet"]]
        if others:
            hit["reject"] = "another contact within quiet"
            continue
        pre = (t >= tk - eg - fw) & (t <= tk - eg)
        post = (t >= tk + eg) & (t <= tk + eg + fw)
        if pre.sum() < h["min_samples"] or post.sum() < h["min_samples"]:
            hit["reject"] = "too few puck samples"
            continue
        (a1, rms1) = _poly(t[pre] - tk, P[pre, 1:3], 1)
        (a2, rms2) = _poly(t[post] - tk, P[post, 1:3], 1)
        da, db = a1[0] - a2[0], a1[1] - a2[1]
        tau = float(np.clip(-(da @ db) / (db @ db), -eg, eg)) if db @ db > 1e-12 else 0.0
        tc = tk + tau
        p1, p2 = a1[0] + a1[1] * tau, a2[0] + a2[1] * tau
        rpre = (R[:, 0] >= tc - eg - fw) & (R[:, 0] <= tc - eg)
        rpost = (R[:, 0] >= tc + eg) & (R[:, 0] <= tc + eg + rw)
        if rpre.sum() < h["min_samples"] or rpost.sum() < h["min_samples"]:
            hit["reject"] = "too few robot samples"
            continue
        (b1, rms3) = _poly(R[rpre, 0] - tc, R[rpre, 1:3], 1)
        (b2, rms4) = _poly(R[rpost, 0] - tc, R[rpost, 1:3], 2)
        pp, pr = (p1 + p2) / 2.0, b1[0]
        dist = float(np.linalg.norm(pp - pr))
        n = (pp - pr) / dist
        vp0, vp1, vr0, vr1 = a1[1], a2[1], b1[1], b2[1]
        tmask = lambda m, A: (A[m, 0], A[m, 3], A[m, 4])
        wp0 = _yaw_rate(*tmask(pre, P), tk, 1, cfg)
        wp1 = _yaw_rate(*tmask(post, P), tk, 1, cfg)
        wr0 = _yaw_rate(*tmask(rpre, R), tc, 1, cfg)
        wr1 = _yaw_rate(*tmask(rpost, R), tc, 2, cfg)
        un0, un1 = float((vp0 - vr0) @ n), float((vp1 - vr1) @ n)
        tv = np.array([-n[1], n[0]])
        hit.update(t=tc - t0, contact_distance=dist, n=n.tolist(), miss=float(np.linalg.norm(p1 - p2)),
                   fit_rms_mm=[1000 * x for x in (rms1, rms2, rms3, rms4)],
                   v_puck_pre=vp0.tolist(), v_puck_post=vp1.tolist(), w_puck_pre=wp0, w_puck_post=wp1,
                   v_robot_pre=vr0.tolist(), v_robot_post=vr1.tolist(), w_robot_pre=wr0, w_robot_post=wr1,
                   puck=pp.tolist(), robot=pr.tolist(), speed=float(np.linalg.norm(vp0 - vr0)),
                   # 0 deg: head-on (the wall bags' 90 deg)
                   angle=math.degrees(math.atan2(abs(float((vp0 - vr0) @ tv)), -un0)),
                   e_n=-un1 / un0 if un0 < 0 else None)
        dvp, dvr = float((vp1 - vp0) @ n), float((vr1 - vr0) @ n)
        hit["mass_ratio"] = -dvr / dvp if abs(dvp) > 0.05 else None  # m_p / m_r, normal momentum
        if max(hit["fit_rms_mm"]) > h["max_fit_rms_mm"]:
            hit["reject"] = "line fits not clean (hand, double contact, mocap)"
        elif hit["miss"] > h["max_miss"]:
            hit["reject"] = "puck lines do not meet"
        elif not (un0 < -0.1 and un1 > 0.0):
            hit["reject"] = "not approaching then separating"
        elif abs(dist - r_sum) > h["contact_tolerance"]:
            hit["reject"] = f"contact distance {dist * 1000:.0f} mm"
        elif None in (wp0, wp1, wr0, wr1):
            hit["reject"] = "yaw rate not measured"
    return hits


# ---------------------------------------------------------------- fit

def _residuals(x, hits, free, p0, rc, sv, sw):
    p = {**p0, **dict(zip(free, x))}
    r = []
    for h in hits:
        vp, wp, vr, wr = impact(h["v_puck_pre"], h["w_puck_pre"], h["v_robot_pre"], h["w_robot_pre"], h["n"], p, rc)
        r += list((vp - h["v_puck_post"]) / sv) + list((vr - h["v_robot_post"]) / sv)
        r += [(wp - h["w_puck_post"]) / sw, (wr - h["w_robot_post"]) / sw]
    return np.array(r)


def fit(hits, cfg):
    h = cfg["robot_hits"]
    spec = h["parameters"]
    p0 = {k: float(spec[k]["value"]) for k in PARAMETERS}
    free = [k for k in PARAMETERS if spec[k].get("fit", True)]
    lo = [spec[k]["min"] for k in free]
    hi = [spec[k]["max"] for k in free]
    rc = constants(cfg)
    args = (hits, free, p0, rc, h["sigma_velocity"], h["sigma_yaw_rate"])
    # the stick/slide switch makes the cost flat in mu when every contact sticks: start from several mu
    best = None
    for mu0 in h["mu_starts"]:
        x0 = [np.clip(mu0 if k == "mu" else p0[k], spec[k]["min"], spec[k]["max"]) for k in free]
        r = least_squares(_residuals, x0, bounds=(lo, hi), args=args, loss=h["loss"], f_scale=h["f_scale"])
        if best is None or r.cost < best.cost:
            best = r
    J = best.jac
    dof = max(J.shape[0] - J.shape[1], 1)
    s2 = 2.0 * best.cost / dof
    try:
        std = np.sqrt(np.diag(np.linalg.pinv(J.T @ J)) * s2)
    except np.linalg.LinAlgError:
        std = np.full(len(free), np.nan)
    p = {**p0, **dict(zip(free, best.x))}
    params = {k: {"value": float(p[k]), "fitted": k in free} for k in PARAMETERS}
    for k, s in zip(free, std):
        params[k]["std"] = float(s)
        params[k]["at_bound"] = bool(np.isclose(p[k], spec[k]["min"]) or np.isclose(p[k], spec[k]["max"]))
    res = _residuals(best.x, *args).reshape(-1, 6)
    rms = {"v_puck_mm_s": float(np.sqrt(np.mean(res[:, :2] ** 2)) * 1000 * h["sigma_velocity"]),
           "v_robot_mm_s": float(np.sqrt(np.mean(res[:, 2:4] ** 2)) * 1000 * h["sigma_velocity"]),
           "w_puck_rad_s": float(np.sqrt(np.mean(res[:, 4] ** 2)) * h["sigma_yaw_rate"]),
           "w_robot_rad_s": float(np.sqrt(np.mean(res[:, 5] ** 2)) * h["sigma_yaw_rate"])}
    for hit, rr in zip(hits, res):
        hit["residual_sigma"] = float(np.sqrt(np.mean(rr ** 2)))
    return p, params, rms


# ---------------------------------------------------------------- report and plots

def _r(x, n=4):
    if isinstance(x, (list, tuple, np.ndarray)):
        return [_r(v, n) for v in x]
    return None if x is None else round(float(x), n)


def summary(hits, p, params, rms, cfg):
    rc = constants(cfg)
    used = [h for h in hits if "reject" not in h]
    lines = [f"{len(used)} hits used of {len(hits)} candidates"]
    for k in PARAMETERS:
        v = params[k]
        s = f" +- {v['std']:.3g}" if v.get("std") is not None and np.isfinite(v["std"]) else ""
        lines.append(f"  {k:4} = {v['value']:.4g}{s}" + ("  (at bound)" if v.get("at_bound") else "")
                     + ("  (fixed)" if not v["fitted"] else ""))
    lines.append(f"  -> puck mass {p['m_p'] * 1000:.1f} g (robot {rc['m_r'] * 1000:.1f} g)")
    lines.append("  rms after the impact: puck v {v_puck_mm_s:.0f} mm/s, robot v {v_robot_mm_s:.0f} mm/s, "
                 "puck w {w_puck_rad_s:.2f} rad/s, robot w {w_robot_rad_s:.2f} rad/s".format(**rms))
    en = [h["e_n"] for h in used]
    mr = [h["mass_ratio"] for h in used if h["mass_ratio"] is not None]
    if en:
        lines.append(f"  direct per hit: e_n median {np.median(en):.3f} [{np.percentile(en, 25):.3f}, "
                     f"{np.percentile(en, 75):.3f}], m_p/m_r median {np.median(mr):.3f} "
                     f"-> puck {np.median(mr) * rc['m_r'] * 1000:.1f} g")
    return "\n".join(lines)


def plot(hits, p, cfg, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rc = constants(cfg)
    used = [h for h in hits if "reject" not in h]
    ang = np.array([h["angle"] for h in used])
    fig, ax = plt.subplots(2, 3, figsize=(16, 9))
    ax[0, 0].scatter(ang, [h["e_n"] for h in used], c=[h["speed"] for h in used], cmap="viridis")
    ax[0, 0].axhline(p["e_n"], color="r", label=f"fit {p['e_n']:.3f}")
    ax[0, 0].set(xlabel="impact angle [deg] (0: head-on)", ylabel="e_n (direct)", title="normal restitution")
    ax[0, 0].legend()
    mr = [(h["angle"], h["mass_ratio"]) for h in used if h["mass_ratio"] is not None]
    ax[0, 1].scatter(*zip(*mr)) if mr else None
    ax[0, 1].axhline(p["m_p"] / rc["m_r"], color="r", label=f"fit {p['m_p'] * 1000:.1f} g")
    ax[0, 1].set(xlabel="impact angle [deg]", ylabel="m_p / m_r (momentum)", title="mass ratio")
    ax[0, 1].legend()
    # measured vs predicted post-impact velocities
    pred = [impact(h["v_puck_pre"], h["w_puck_pre"], h["v_robot_pre"], h["w_robot_pre"], h["n"], p, rc)
            for h in used]
    for k, (lab, mi, pi) in enumerate([("puck v", "v_puck_post", 0), ("robot v", "v_robot_post", 2)]):
        m = np.array([h[mi] for h in used]).ravel()
        q = np.array([x[pi] for x in pred]).ravel()
        a = ax[0, 2] if k == 0 else ax[1, 0]
        a.scatter(m, q, s=12)
        lim = [min(m.min(), q.min()), max(m.max(), q.max())]
        a.plot(lim, lim, "k--", lw=0.8)
        a.set(xlabel=f"measured {lab} after [m/s]", ylabel="model [m/s]", title=f"{lab} components after the impact")
    for k, (lab, mi, pi) in enumerate([("puck w", "w_puck_post", 1), ("robot w", "w_robot_post", 3)]):
        a = ax[1, 1 + k]
        m = np.array([h[mi] for h in used])
        q = np.array([x[pi] for x in pred])
        a.scatter(m, q, s=12)
        lim = [min(m.min(), q.min()), max(m.max(), q.max())]
        a.plot(lim, lim, "k--", lw=0.8)
        a.set(xlabel=f"measured {lab} after [rad/s]", ylabel="model [rad/s]", title=f"{lab} after the impact")
    fig.tight_layout()
    fig.savefig(path, dpi=90)
    plt.close(fig)


def run(bags, cfg, out=None, save_plot=True):
    hits = []
    out_dirs = []
    for bag in bags:
        raw, out_dir = load(bag, cfg)
        hb = measure(raw, cfg)
        for h in hb:
            h["bag"] = os.path.basename(out_dir.rstrip("/"))
        out_dirs.append(out_dir)
        n_used = sum("reject" not in h for h in hb)
        print(f"{os.path.basename(out_dir.rstrip('/'))}: {len(hb)} candidate contacts, {n_used} used")
        for h in hb:
            if "reject" in h:
                print(f"   t={h['t']:7.2f}  rejected: {h['reject']}")
        hits += hb
    used = [h for h in hits if "reject" not in h]
    if len(used) < 3:
        raise RuntimeError(f"only {len(used)} usable hits")
    p, params, rms = fit(used, cfg)
    text = summary(hits, p, params, rms, cfg)
    if out is None:
        out = os.path.join(out_dirs[0], "hits.yaml") if len(out_dirs) == 1 else \
            os.path.join(os.path.commonpath(out_dirs), "hits.yaml")
    doc = {"bags": out_dirs, "parameters": params, "rms": rms, "robot": cfg["robot_hits"]["robot"],
           "puck": cfg["puck"],
           "hits": [{k: _r(v) if k not in ("bag", "reject") else v for k, v in h.items()} for h in hits]}
    data.save_yaml(out, doc, header="Puck-robot impacts (identify.py hits): measured hits and the fitted impact law.")
    if save_plot:
        plot(hits, p, cfg, os.path.splitext(out)[0] + ".png")
    print(text)
    print(f"wrote {out}")
    return p, params, hits
