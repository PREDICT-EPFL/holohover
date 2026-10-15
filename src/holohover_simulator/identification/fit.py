"""Output-error fit of the puck model to the segments.

Each segment is simulated from its own initial state (6 unknowns per segment, fitted together
with the model) and compared with the measured position and yaw. All segments share the model
parameters and the wall offsets. The fit runs in two stages: first only the initial states
(model at its initial guess), then everything.
"""
import time

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix

import model
import segment as seg


def _wrap(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


class Problem:
    def __init__(self, rec, segments, walls, cfg, events=()):
        f = cfg["fit"]
        self.cfg = cfg
        self.radius = cfg["puck"]["radius"]
        self.kappa = cfg["puck"]["kappa"]
        self.sig_p, self.sig_y = f["sigma_position"], f["sigma_yaw"]
        self.sig_v, self.sig_w = f["sigma_velocity"], f["sigma_yaw_rate"]
        # EKF velocities are measurements too, except while the filter settles after a contact
        self.use_velocity = rec.measured_velocity
        contacts = np.array([e["t"] for e in events if e["kind"] in ("wall", "hit")])
        spec = f["parameters"]
        self.p0 = {k: float(spec[k]["value"]) for k in model.PARAMETERS}
        self.free = [k for k in model.PARAMETERS if spec[k].get("fit", True)]
        self.walls = [dict(w) for w in walls]
        impact_walls = set()
        self.segs = []
        for g in segments:
            idx = rec.window(g["t0"], g["t1"])[:: max(1, int(f["decimate"]))]
            if idx.size < 10:
                continue
            ok = rec.yaw_ok[idx]
            s = {**g, "idx": idx, "t": rec.t[idx] - g["t0"], "p": rec.p[idx],
                 "yaw": rec.yaw[idx], "yaw_ok": ok, "x0": initial_state(rec, g, contacts)}
            if self.use_velocity:
                t = rec.t[idx]
                p_ok = np.ones(idx.size, bool)
                v_ok = np.isfinite(rec.v[idx, 0])
                for tc in contacts:
                    p_ok &= ~((t >= tc - 0.005) & (t <= tc + f["position_settle"]))
                    v_ok &= ~((t >= tc - 0.01) & (t <= tc + f["velocity_settle"]))
                s.update(p_ok=p_ok, v=rec.v[idx], w=rec.w[idx], v_ok=v_ok,
                         w_ok=v_ok & ok & np.isfinite(rec.w[idx]))
            else:
                s["p_ok"] = np.ones(idx.size, bool)
            self.segs.append(s)
            if g["label"] == "wall_impact":
                impact_walls |= {e["wall"] for e in events
                                 if e["kind"] == "wall" and g["t0"] <= e["t"] <= g["t1"]}
        self.wall_free = [i for i, w in enumerate(self.walls) if f["fit_walls"] and w["name"] in impact_walls]
        self.ng = len(self.free) + len(self.wall_free)
        self.rows = [2 * int(s["p_ok"].sum()) + int(s["yaw_ok"].sum())
                     + (2 * int(s["v_ok"].sum()) + int(s["w_ok"].sum()) if self.use_velocity else 0)
                     for s in self.segs]

    # parameter vector: [free model parameters, free wall offsets, initial states]
    def pack(self, p, walls, x0s):
        return np.concatenate([[p[k] for k in self.free], [walls[i]["offset"] for i in self.wall_free],
                               np.concatenate(x0s)])

    def unpack(self, x):
        p = dict(self.p0)
        p.update(zip(self.free, x[: len(self.free)]))
        walls = [dict(w) for w in self.walls]
        for j, i in enumerate(self.wall_free):
            walls[i]["offset"] = float(x[len(self.free) + j])
        x0s = x[self.ng:].reshape(-1, 6)
        return p, walls, x0s

    def predict(self, s, p, walls, x0, return_impacts=False):
        return model.rollout(x0, s["t"], p, walls, self.radius, self.kappa, return_impacts)

    def residuals(self, x):
        p, walls, x0s = self.unpack(x)
        out = []
        for s, x0 in zip(self.segs, x0s):
            st = self.predict(s, p, walls, x0)
            out.append(((st[s["p_ok"], 0:2] - s["p"][s["p_ok"]]) / self.sig_p).ravel())
            out.append(_wrap(st[s["yaw_ok"], 2] - s["yaw"][s["yaw_ok"]]) / self.sig_y)
            if self.use_velocity:
                out.append(((st[s["v_ok"], 3:5] - s["v"][s["v_ok"]]) / self.sig_v).ravel())
                out.append((st[s["w_ok"], 5] - s["w"][s["w_ok"]]) / self.sig_w)
        return np.concatenate(out)

    def sparsity(self, globals_free=True):
        m, n = sum(self.rows), self.ng + 6 * len(self.segs)
        A = lil_matrix((m, n), dtype=int)
        r = 0
        for j, rows in enumerate(self.rows):
            if globals_free:
                A[r:r + rows, : self.ng] = 1
            A[r:r + rows, self.ng + 6 * j: self.ng + 6 * j + 6] = 1
            r += rows
        return A

    def bounds(self):
        spec = self.cfg["fit"]["parameters"]
        lo = [spec[k]["min"] for k in self.free] + [-np.inf] * (len(self.wall_free) + 6 * len(self.segs))
        hi = [spec[k]["max"] for k in self.free] + [np.inf] * (len(self.wall_free) + 6 * len(self.segs))
        return np.array(lo, float), np.array(hi, float)


def initial_state(rec, g, contacts=(), window=0.06, yaw_window=0.25):
    """State at the segment start from line fits over its first `window` seconds (position)
    and up to `yaw_window` seconds before the first contact (yaw: the fit compares wrapped
    angles, a wrong initial yaw rate can end in an aliased solution)."""
    idx = rec.window(g["t0"], g["t0"] + min(window, (g["t1"] - g["t0"]) / 3.0))
    if idx.size < 4:
        return rec.state(rec.window(g["t0"], g["t1"])[0])
    a, b = seg._line(rec.t[idx] - g["t0"], rec.p[idx])
    t_end = min([g["t0"] + yaw_window, (g["t0"] + g["t1"]) / 2.0]
                + [tc - 0.01 for tc in contacts if g["t0"] < tc < g["t1"]])
    idy = rec.window(g["t0"], max(t_end, g["t0"] + window))
    ok = idy[rec.yaw_ok[idy]]
    if ok.size >= 4:
        ya, yb = seg._line(rec.t[ok] - g["t0"], rec.yaw_u[ok])
    else:
        ya, yb = rec.yaw_u[idx[0]], rec.w[idx[0]]
    return np.array([a[0], a[1], ya, b[0], b[1], yb], dtype=float)


def run(rec, seg_yaml, cfg, verbose=1):
    walls = model.walls_from_yaml(seg_yaml["walls"])
    segments = [g for g in seg_yaml["segments"] if g["label"] in seg.FIT_LABELS]
    if not segments:
        raise RuntimeError("no segments to fit")
    pr = Problem(rec, segments, walls, cfg, seg_yaml.get("events", []))
    f = cfg["fit"]
    x = pr.pack(pr.p0, pr.walls, [s["x0"] for s in pr.segs])
    lo, hi = pr.bounds()
    x = np.clip(x, lo, hi)
    print(f"fitting {len(pr.segs)} segments ({cfg['source']} data"
          f"{', with velocities' if pr.use_velocity else ''}), {sum(pr.rows)} residuals, model parameters {pr.free}, "
          f"wall offsets {[pr.walls[i]['name'] for i in pr.wall_free]}")
    t_start = time.time()
    # stage 1: initial states only
    free = np.zeros(x.size, bool)
    free[pr.ng:] = True

    def sub(xf):
        xx = x.copy()
        xx[free] = xf
        return pr.residuals(xx)

    r1 = least_squares(sub, x[free], jac_sparsity=pr.sparsity(False).tocsr()[:, np.flatnonzero(free)], x_scale="jac",
                       loss=f["loss"], f_scale=f["f_scale"], max_nfev=f["max_nfev"], verbose=verbose)
    x[free] = r1.x
    print(f"stage 1 (initial states): cost {r1.cost:.4g}")
    # stage 2: everything
    res = least_squares(pr.residuals, x, jac_sparsity=pr.sparsity(True), bounds=(lo, hi), x_scale="jac",
                        loss=f["loss"], f_scale=f["f_scale"], max_nfev=f["max_nfev"], verbose=verbose)
    print(f"stage 2 (all): cost {res.cost:.4g}, {res.message} ({time.time() - t_start:.0f} s)")
    return pr, res


def _std(pr, res):
    """1-sigma of the model parameters and wall offsets from the Gauss-Newton covariance."""
    J = res.jac
    JtJ = (J.T @ J).toarray() if hasattr(J, "toarray") else J.T @ J
    m, n = res.fun.size, res.x.size
    s2 = 2.0 * res.cost / max(m - n, 1)
    # parameters at a bound are not free: leave them out of the inversion
    at_bound = res.active_mask != 0
    keep = np.flatnonzero(~at_bound)
    std = np.full(n, np.nan)
    try:
        cov = np.linalg.inv(JtJ[np.ix_(keep, keep)]) * s2
        std[keep] = np.sqrt(np.clip(np.diag(cov), 0.0, None))
    except np.linalg.LinAlgError:
        pass
    return std, at_bound


def report(pr, res):
    p, walls, x0s = pr.unpack(res.x)
    std, at_bound = _std(pr, res)
    spec = pr.cfg["fit"]["parameters"]
    params = {}
    for k in model.PARAMETERS:
        if k in pr.free:
            j = pr.free.index(k)
            params[k] = {"value": float(p[k]), "std": None if np.isnan(std[j]) else float(std[j]),
                         "fitted": True, "at_bound": bool(at_bound[j])}
        else:
            params[k] = {"value": float(spec[k]["value"]), "std": None, "fitted": False}
    for j, i in enumerate(pr.wall_free):
        sd = std[len(pr.free) + j]
        walls[i]["offset_std"] = None if np.isnan(sd) else float(sd)
    seg_out, all_p, all_y, all_v = [], [], [], []
    by_label = {}
    for s, x0 in zip(pr.segs, x0s):
        st, imp = pr.predict(s, p, walls, x0, return_impacts=True)
        ep = np.linalg.norm(st[s["p_ok"], 0:2] - s["p"][s["p_ok"]], axis=1)
        ey = _wrap(st[s["yaw_ok"], 2] - s["yaw"][s["yaw_ok"]])
        all_p.append(ep)
        all_y.append(ey)
        ev = None
        if pr.use_velocity:
            ev = np.linalg.norm(st[s["v_ok"], 3:5] - s["v"][s["v_ok"]], axis=1)
            all_v.append(ev)
        by_label.setdefault(s["label"], []).append(ep)
        seg_out.append({"id": s["id"], "label": s["label"], "t0": s["t0"], "t1": s["t1"],
                        "state0": [round(float(c), 6) for c in x0],
                        "rms_position_mm": round(1e3 * float(np.sqrt(np.mean(ep ** 2))), 3),
                        "rms_yaw_deg": round(float(np.degrees(np.sqrt(np.mean(ey ** 2)))) if ey.size else 0.0, 3),
                        "model_impacts": len(imp),
                        **({"rms_velocity_mm_s": round(1e3 * float(np.sqrt(np.mean(ev ** 2))), 2) if ev.size else None}
                           if ev is not None else {})})
    all_p, all_y = np.concatenate(all_p), np.concatenate(all_y)
    rms = lambda e: round(float(np.sqrt(np.mean(e ** 2))), 6)
    out = {
        "parameters": params,
        "puck": {"radius": pr.radius, "kappa": pr.kappa},
        "walls": [{**w2, **({"offset_std": w["offset_std"]} if w.get("offset_std") is not None else {})}
                  for w, w2 in zip(walls, model.walls_to_yaml(walls))],
        "fit": {"segments": len(pr.segs), "residuals": int(res.fun.size), "cost": float(res.cost),
                "message": res.message, "success": bool(res.success),
                "rms_position_mm": round(1e3 * rms(all_p), 3),
                "rms_position_mm_by_label": {k: round(1e3 * rms(np.concatenate(v)), 3) for k, v in by_label.items()},
                "rms_yaw_deg": round(float(np.degrees(rms(all_y))), 3),
                **({"rms_velocity_mm_s": round(1e3 * rms(np.concatenate(all_v)), 2)} if all_v else {}),
                "data": pr.cfg["source"]},
        "segments": seg_out,
    }
    return out


def summary_text(out):
    lines = ["model parameters (1 sigma):"]
    units = {"c_v": "1/s", "g_x": "m/s^2", "g_y": "m/s^2", "c_w": "1/s", "c_wc": "rad/s^2", "e_n": "", "mu": "", "e_t": ""}
    for k, v in out["parameters"].items():
        sd = f" +- {v['std']:.3g}" if v.get("std") is not None else ""
        flag = "" if v["fitted"] else "  (fixed)"
        flag += "  (at bound)" if v.get("at_bound") else ""
        lines.append(f"  {k:5s} = {v['value']:.4g}{sd} {units[k]}{flag}")
    f = out["fit"]
    vel = f" velocity {f['rms_velocity_mm_s']} mm/s," if "rms_velocity_mm_s" in f else ""
    lines.append(f"rms error ({f['data']} data): position {f['rms_position_mm']} mm {f['rms_position_mm_by_label']},"
                 f"{vel} yaw {f['rms_yaw_deg']} deg")
    p = {k: v["value"] for k, v in out["parameters"].items()}
    lines += [
        "for the simulators (they model less than this fit, see README):",
        f"  python-sim config.json  table.friction ~ c_v = {p['c_v']:.4g}   shot_mpc.restitution ~ e_n = {p['e_n']:.3g}",
        f"  C++ simulator (Box2D)   linearDamping ~ c_v = {p['c_v']:.4g}, angularDamping ~ c_w = {p['c_w']:.4g}, "
        f"restitution ~ e_n = {p['e_n']:.3g}, friction ~ mu = {p['mu']:.3g}",
    ]
    return "\n".join(lines)
