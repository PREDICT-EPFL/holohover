"""Measured data vs model prediction.

Three windows (or PNGs with --save):
  segments  one segment at a time: measured (dots) and the model simulated from the fitted
            initial state (line) for position, velocity, yaw and yaw rate, the position error,
            and the path on the table. Keys: n / b next / previous segment, h toggles the
            predictions started every `every` seconds from the measured state.
  timeline  the whole recording with the model predictions over `horizon`, started every
            `every` seconds from the measured state (the EKF state, or for the raw pose a fit
            of past samples only, as an online predictor would). Zoom with the toolbar.
  summary   impact law against every measured impact, glide accelerations and spin decay
            against the model, and the prediction error against the horizon.
"""
import os

import matplotlib.pyplot as plt
import numpy as np

import model
import segment as seg

C_MEAS = "0.25"
C_MODEL = "tab:red"
C_PRED = "tab:blue"


def _wrap(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


class Results:
    def __init__(self, rec, seg_yaml, res, cfg):
        self.rec, self.cfg = rec, cfg
        self.p = {k: v["value"] for k, v in res["parameters"].items()}
        self.walls = model.walls_from_yaml(res["walls"])
        self.radius, self.kappa = res["puck"]["radius"], res["puck"]["kappa"]
        self.segments = res["segments"]
        self.events = seg_yaml.get("events", [])
        current = [(g["t0"], g["t1"]) for g in seg_yaml["segments"]]
        if current != [(g["t0"], g["t1"]) for g in self.segments]:
            print("warning: segments.yaml changed since the fit, showing the fitted segments (run fit again)")
        # contiguous segments (glide - impact - glide) form runs, a prediction may cross segments
        runs = []
        for g in sorted(self.segments, key=lambda g: g["t0"]):
            if runs and abs(g["t0"] - runs[-1][1]) < 1e-6:
                runs[-1][1] = g["t1"]
            else:
                runs.append([g["t0"], g["t1"]])
        self.runs = runs

    def simulate(self, x0, t):
        return model.rollout(x0, t, self.p, self.walls, self.radius, self.kappa, return_impacts=True)

    def causal_state(self, t, window=0.06):
        """Measured state at time t, from past samples only: the EKF state (state recordings)
        or line fits over the last `window` seconds (raw pose)."""
        r = self.rec
        if r.measured_velocity:
            i = int(np.searchsorted(r.t, t, side="right")) - 1
            if i < 0 or t - r.t[i] > 0.02 or not r.yaw_ok[i] or not np.isfinite(r.v[i, 0]):
                return None
            # last EKF sample, propagated to t (at most 20 ms)
            return model.flight(r.state(i), [t - r.t[i]], self.p)[0]
        idx = r.window(t - window, t)
        if idx.size < 6 or r.t[idx[-1]] - r.t[idx[0]] < 0.6 * window:
            return None
        a, b = seg._line(r.t[idx] - t, r.p[idx])
        ok = idx[r.yaw_ok[idx]]
        if ok.size < 6:
            return None
        ya, yb = seg._line(r.t[ok] - t, r.yaw_u[ok])
        return np.array([a[0], a[1], ya, b[0], b[1], yb])

    def run_of(self, t):
        for a, b in self.runs:
            if a <= t <= b:
                return a, b
        return None

    def predictions(self, t0, t1, horizon, every, inside_runs=True, window=0.06):
        """[(t_start, times, states)] for predictions started in [t0, t1].

        Starts whose state estimation window contains an impact or a hit are skipped: the line
        fit would average the velocities before and after it."""
        out = []
        contacts = np.array([e["t"] for e in self.events if e["kind"] in ("wall", "hit")])
        for ts in np.arange(t0, t1, every):
            if contacts.size and np.any((contacts > ts - window - 0.01) & (contacts <= ts)):
                continue
            end = ts + horizon
            if inside_runs:
                run = self.run_of(ts - 0.06)
                if run is None or ts > run[1]:
                    continue
                end = min(end, run[1])
            x0 = self.causal_state(ts, window)
            if x0 is None or end <= ts:
                continue
            tt = np.linspace(0.0, end - ts, max(2, int((end - ts) / 0.005)))
            out.append((ts, ts + tt, self.simulate(x0, tt)[0]))
        return out


# ------------------------------------------------------------------------------------- segments
class SegmentView:
    def __init__(self, R, start, horizon, every):
        self.R, self.horizon, self.every = R, horizon, every
        self.i = 0
        if start is not None:
            ids = [g["id"] for g in R.segments]
            self.i = ids.index(start) if start in ids else 0
        self.fan = False
        self.fig = plt.figure(figsize=(16, 9.5))
        gs = self.fig.add_gridspec(4, 3, width_ratios=[1, 1, 1.1], hspace=0.35, wspace=0.25,
                                   left=0.05, right=0.98, top=0.92, bottom=0.06)
        f = self.fig
        self.ax = {"x": f.add_subplot(gs[0, 0]), "y": f.add_subplot(gs[0, 1]),
                   "vx": f.add_subplot(gs[1, 0]), "vy": f.add_subplot(gs[1, 1]),
                   "yaw": f.add_subplot(gs[2, 0]), "w": f.add_subplot(gs[2, 1]),
                   "err": f.add_subplot(gs[3, 0:2]), "xy": f.add_subplot(gs[:, 2])}
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)

    def on_key(self, e):
        if e.key == "n":
            self.i = (self.i + 1) % len(self.R.segments)
        elif e.key == "b":
            self.i = (self.i - 1) % len(self.R.segments)
        elif e.key == "h":
            self.fan = not self.fan
        else:
            return
        self.draw()
        self.fig.canvas.draw_idle()

    def draw(self):
        R, r = self.R, self.R.rec
        g = R.segments[self.i]
        for a in self.ax.values():
            a.clear()
        pad = 0.15
        idx = r.window(g["t0"], g["t1"])
        ctx = r.window(g["t0"] - pad, g["t1"] + pad)
        t_rel = r.t[idx] - g["t0"]
        tt = np.linspace(0.0, g["t1"] - g["t0"], max(50, int((g["t1"] - g["t0"]) / 0.002)))
        st, imp = R.simulate(np.array(g["state0"]), tt)
        st_m = R.simulate(np.array(g["state0"]), t_rel)[0]
        tm = g["t0"] + tt

        def meas(ax, y, ok=None, ylabel=""):
            sl = ctx if ok is None else ctx[ok[ctx]]
            ax.plot(r.t[sl], y[sl], ".", ms=2, color=C_MEAS, label="measured")
            ax.set_ylabel(ylabel)
            ax.axvspan(g["t0"], g["t1"], color="0.9", zorder=-1)

        vlab = "(EKF)" if r.measured_velocity else "(smoothed)"
        meas(self.ax["x"], r.p[:, 0], ylabel="x [m]")
        meas(self.ax["y"], r.p[:, 1], ylabel="y [m]")
        meas(self.ax["vx"], r.v[:, 0], ylabel=f"vx [m/s] {vlab}")
        meas(self.ax["vy"], r.v[:, 1], ylabel=f"vy [m/s] {vlab}")
        meas(self.ax["yaw"], r.yaw_u, ok=r.yaw_ok, ylabel="yaw [rad] (unwrapped)")
        meas(self.ax["w"], r.w, ok=r.yaw_ok, ylabel=f"yaw rate [rad/s] {vlab}")
        for key, col in (("x", 0), ("y", 1), ("vx", 3), ("vy", 4), ("w", 5)):
            self.ax[key].plot(tm, st[:, col], color=C_MODEL, lw=1.5, label="model")
        # the fit compares wrapped angles: shift the model yaw by whole turns onto the measurement
        ok = r.yaw_ok[idx]
        shift = 0.0
        if ok.any():
            shift = 2 * np.pi * np.round(np.median(r.yaw_u[idx][ok] - st_m[ok, 2]) / (2 * np.pi))
        self.ax["yaw"].plot(tm, st[:, 2] + shift, color=C_MODEL, lw=1.5, label="model")
        for t_hit, _ in imp:
            for a in self.ax.values():
                if a is not self.ax["xy"]:
                    a.axvline(g["t0"] + t_hit, color=C_MODEL, ls=":", lw=1)
        err = 1e3 * np.linalg.norm(st_m[:, 0:2] - r.p[idx], axis=1)
        self.ax["err"].plot(r.t[idx], err, color=C_MODEL, lw=1)
        self.ax["err"].set_ylabel("position error [mm]")
        self.ax["err"].set_xlabel("time [s]")
        self.ax["err"].axvspan(g["t0"], g["t1"], color="0.9", zorder=-1)
        # table
        axy = self.ax["xy"]
        axy.plot(r.p[ctx, 0], r.p[ctx, 1], ".", ms=2, color=C_MEAS, label="measured")
        axy.plot(st[:, 0], st[:, 1], color=C_MODEL, lw=1.5, label="model")
        axy.plot(st[0, 0], st[0, 1], "o", color=C_MODEL, ms=5)
        _walls(axy, R.walls, np.vstack([r.p[ctx], st[:, 0:2]]))
        axy.set_aspect("equal", adjustable="datalim")
        axy.set_xlabel("x [m]")
        axy.set_ylabel("y [m]")
        if self.fan:
            for ts, tp, sp in R.predictions(g["t0"] - pad, g["t1"] + pad, self.horizon, self.every):
                for key, col in (("x", 0), ("y", 1), ("vx", 3), ("vy", 4), ("w", 5)):
                    self.ax[key].plot(tp, sp[:, col], color=C_PRED, lw=0.7, alpha=0.7)
                axy.plot(sp[:, 0], sp[:, 1], color=C_PRED, lw=0.7, alpha=0.7)
                axy.plot(sp[0, 0], sp[0, 1], ".", color=C_PRED, ms=3)
        for key in ("x", "y", "vx", "vy", "yaw", "w", "err"):
            self.ax[key].set_xlim(r.t[ctx[0]], r.t[ctx[-1]])
        self.ax["x"].legend(loc="best", fontsize=8)
        axy.legend(loc="best", fontsize=8)
        fan = f"   blue: predictions over {self.horizon:.2f} s from the measured state every {self.every:.2f} s" if self.fan else ""
        self.fig.suptitle(f"segment {g['id']} ({self.i + 1}/{len(R.segments)}): {g['label']}, "
                          f"{g['t0']:.2f}-{g['t1']:.2f} s, rms position {g['rms_position_mm']:.2f} mm, "
                          f"yaw {g['rms_yaw_deg']:.2f} deg, model impacts {len(imp)}{fan}\n"
                          "keys: n / b next / previous segment, h predictions from every time", fontsize=10)


def _walls(ax, walls, pts):
    lo, hi = pts.min(0) - 0.05, pts.max(0) + 0.05
    for w in walls:
        n, d = w["normal"], w["offset"]
        t = np.array([-n[1], n[0]])
        corners = np.array([lo, hi, [lo[0], hi[1]], [hi[0], lo[1]]]) @ t
        a = np.outer([corners.min(), corners.max()], t) + d * n
        # only draw walls near the shown area
        if np.min(np.abs(pts @ n - d)) < 0.2:
            ax.plot(a[:, 0], a[:, 1], color="tab:blue", lw=1.5)


# ------------------------------------------------------------------------------------- timeline
def timeline(R, horizon, every, t_range=None):
    r = R.rec
    fig, axs = plt.subplots(3, 1, figsize=(16, 9), sharex=True)
    t0, t1 = t_range if t_range else (r.t[0], r.t[-1])
    sl = r.window(t0, t1)
    for ax, y, lab in zip(axs, (r.p[:, 0], r.p[:, 1]), ("x [m]", "y [m]")):
        ax.plot(r.t[sl], y[sl], ".", ms=1.5, color=C_MEAS)
        ax.set_ylabel(lab)
    preds = R.predictions(t0, t1, horizon, every, inside_runs=False)
    err_t, err_v = [], []
    for ts, tp, sp in preds:
        axs[0].plot(tp, sp[:, 0], color=C_PRED, lw=0.7)
        axs[1].plot(tp, sp[:, 1], color=C_PRED, lw=0.7)
        axs[0].plot(tp[0], sp[0, 0], ".", color=C_PRED, ms=2)
        axs[1].plot(tp[0], sp[0, 1], ".", color=C_PRED, ms=2)
        idx = r.window(tp[0], tp[-1])
        if idx.size:
            e = np.linalg.norm(np.column_stack([np.interp(r.t[idx], tp, sp[:, 0]),
                                                np.interp(r.t[idx], tp, sp[:, 1])]) - r.p[idx], axis=1)
            err_t.append(ts)
            err_v.append(1e3 * e[-1])
    axs[2].plot(err_t, err_v, ".", color=C_PRED, ms=3)
    axs[2].set_yscale("log")
    axs[2].set_ylabel(f"error at the end of the\nprediction [mm]")
    axs[2].set_xlabel("time [s]")
    for g in R.segments:
        for ax in axs:
            ax.axvspan(g["t0"], g["t1"], color="tab:green" if g["label"] == "glide" else "tab:orange",
                       alpha=0.12, lw=0)
    for e in R.events:
        if e["kind"] in ("wall", "hit"):
            for ax in axs:
                ax.axvline(e["t"], color="tab:red" if e["kind"] == "hit" else "0.6", lw=0.6,
                           ls="-" if e["kind"] == "hit" else ":")
    axs[0].set_xlim(t0, t1)
    fig.suptitle(f"measured (grey) and model predictions over {horizon:.2f} s started every {every:.2f} s "
                 "from the measured state (blue); red lines: hits (not modelled)", fontsize=10)
    fig.tight_layout()
    return fig


# ------------------------------------------------------------------------------------- summary
def _glide_stats(R):
    """Per glide (> 0.4 s): mean velocity, constant acceleration and spin decay from fits."""
    r = R.rec
    rows = []
    for g in R.segments:
        if g["label"] != "glide" or g["t1"] - g["t0"] < 0.4:
            continue
        idx = r.window(g["t0"], g["t1"])
        t = r.t[idx] - r.t[idx].mean()
        A = np.column_stack([np.ones_like(t), t, 0.5 * t * t])
        c, *_ = np.linalg.lstsq(A, r.p[idx], rcond=None)
        ok = idx[r.yaw_ok[idx]]
        w = dw = np.nan
        if ok.size > 20:
            ty = r.t[ok] - r.t[idx].mean()
            cy, *_ = np.linalg.lstsq(np.column_stack([np.ones_like(ty), ty, 0.5 * ty * ty]), r.yaw_u[ok], rcond=None)
            w, dw = cy[1], cy[2]
        rows.append({"v": c[1], "a": c[2], "w": w, "dw": dw})
    return rows


def _impact_stats(R):
    rows = []
    walls = {w["name"]: w for w in R.walls}
    for e in R.events:
        if e["kind"] != "wall" or e.get("w_pre") is None or e.get("w_post") is None or e["wall"] not in walls:
            continue
        n = walls[e["wall"]]["normal"]
        t = np.array([-n[1], n[0]])
        v1, v2 = np.array(e["v_pre"]), np.array(e["v_post"])
        vn1, vn2 = v1 @ n, v2 @ n
        ut1 = v1 @ t - R.radius * e["w_pre"]
        ut2 = v2 @ t - R.radius * e["w_post"]
        if vn1 >= -0.05:
            continue
        along = e["contact"][0] if abs(n[1]) > abs(n[0]) else e["contact"][1]
        rows.append({"t": e["t"], "wall": e["wall"], "along": along,
                     "vn1": vn1, "e_n": -vn2 / vn1, "angle": np.degrees(np.arctan2(abs(v1 @ t), -vn1)),
                     "speed": np.linalg.norm(v1), "psi": abs(ut1) / -vn1,
                     "ratio": ut2 / ut1 if abs(ut1) > 0.02 else np.nan})
    return rows


def summary(R, horizon):
    p = R.p
    fig, axs = plt.subplots(2, 3, figsize=(16, 9.5))
    fig.subplots_adjust(left=0.06, right=0.98, top=0.9, bottom=0.07, hspace=0.35, wspace=0.28)
    imp = _impact_stats(R)
    # (a) normal restitution: where on the wall, at which angle
    ax = axs[0, 0]
    sc = None
    for k, name in enumerate(sorted({i["wall"] for i in imp})):
        sub = [i for i in imp if i["wall"] == name]
        sc = ax.scatter([i["along"] for i in sub], [i["e_n"] for i in sub], c=[i["angle"] for i in sub],
                        cmap="viridis", vmin=0, vmax=60, s=22, marker="os^v"[k % 4], label=name)
    if sc is not None:
        fig.colorbar(sc, ax=ax, label="incidence angle from the normal [deg]")
    ax.axhline(p["e_n"], color=C_MODEL, label=f"model e_n = {p['e_n']:.3f}")
    ax.set_xlabel("contact position along the wall [m] (x for the y walls)")
    ax.set_ylabel("-v_n' / v_n")
    ax.set_title("normal restitution, one marker per impact", fontsize=10)
    ax.legend(fontsize=8)
    # (b) tangential: Maw-style diagram
    ax = axs[0, 1]
    psi = np.array([i["psi"] for i in imp]) if imp else np.empty(0)
    ratio = np.array([i["ratio"] for i in imp]) if imp else np.empty(0)
    ax.plot(psi, ratio, "o", ms=4, color=C_MEAS, label="measured")
    xs = np.linspace(1e-3, max(1.0, np.nanmax(psi) * 1.1 if psi.size else 1.0), 300)
    ax.plot(xs, model.tangential_ratio(xs, p["e_n"], p["mu"], p["e_t"], R.kappa), color=C_MODEL,
            label=f"model mu = {p['mu']:.3f}, e_t = {p['e_t']:.3f}")
    ax.axhline(-p["e_t"], color=C_MODEL, ls=":", lw=0.8)
    ax.set_xlabel("|u_t| / |v_n| before the impact  (u_t = v_t - r w, contact slip)")
    ax.set_ylabel("u_t' / u_t")
    ax.set_ylim(-1.1, 1.1)
    ax.set_title("tangential: stick (left, -e_t) to slide (right)", fontsize=10)
    ax.legend(fontsize=8)
    # (c) glide accelerations: measured vs model
    gl = _glide_stats(R)
    ax = axs[0, 2]
    if gl:
        g = np.array([p["g_x"], p["g_y"]])
        meas = np.array([row["a"] for row in gl])
        pred = np.array([g - p["c_v"] * row["v"] for row in gl])
        ax.plot(1e3 * pred[:, 0], 1e3 * meas[:, 0], "o", ms=4, label="a_x")
        ax.plot(1e3 * pred[:, 1], 1e3 * meas[:, 1], "s", ms=4, label="a_y")
        lim = 1e3 * np.nanmax(np.abs(np.concatenate([meas.ravel(), pred.ravel()]))) * 1.1
        ax.plot([-lim, lim], [-lim, lim], "k:", lw=0.8)
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
    ax.set_xlabel("model  g - c_v v  [mm/s^2]")
    ax.set_ylabel("measured (quadratic fit per glide) [mm/s^2]")
    ax.set_title(f"glide acceleration: c_v = {p['c_v']:.4f} 1/s, g = ({1e3 * p['g_x']:.1f}, {1e3 * p['g_y']:.1f}) mm/s^2",
                 fontsize=10)
    ax.legend(fontsize=8)
    # (d) spin decay
    ax = axs[1, 0]
    if gl:
        w = np.array([row["w"] for row in gl])
        dw = np.array([row["dw"] for row in gl])
        ax.plot(w, dw, "o", ms=4, color=C_MEAS, label="measured (per glide)")
        ws = np.linspace(np.nanmin(w) - 1, np.nanmax(w) + 1, 200)
        ax.plot(ws, -p["c_w"] * ws - p["c_wc"] * np.sign(ws), color=C_MODEL,
                label=f"model c_w = {p['c_w']:.3f}, c_wc = {p['c_wc']:.3f}")
    ax.set_xlabel("yaw rate [rad/s]")
    ax.set_ylabel("yaw acceleration [rad/s^2]")
    ax.set_title("spin decay", fontsize=10)
    ax.legend(fontsize=8)
    # (e) prediction error vs horizon
    ax = axs[1, 1]
    hs, rms, p90 = prediction_error(R, horizon)
    ax.plot(hs, rms, color=C_PRED, label="rms")
    ax.plot(hs, p90, color=C_PRED, ls="--", label="90th percentile")
    ax.set_xlabel("prediction horizon [s]")
    ax.set_ylabel("position error [mm]")
    ax.set_title("prediction from the measured state, inside the segments", fontsize=10)
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    # (f) per segment rms
    ax = axs[1, 2]
    ids = [g["id"] for g in R.segments]
    ax.bar(range(len(ids)), [g["rms_position_mm"] for g in R.segments],
           color=["tab:green" if g["label"] == "glide" else "tab:orange" for g in R.segments])
    ax.set_xlabel("segment (green: glide, orange: wall impact)")
    ax.set_ylabel("rms position error [mm]")
    ax.set_title("fit error per segment", fontsize=10)
    fig.suptitle("puck model: " + ", ".join(f"{k} = {v:.4g}" for k, v in p.items()), fontsize=11)
    return fig


def prediction_error(R, horizon, every=0.02):
    hs = np.linspace(0.0, horizon, 26)
    errs = [[] for _ in hs]
    for a, b in R.runs:
        for ts, tp, sp in R.predictions(a + 0.06, b, horizon, every):
            idx = R.rec.window(tp[0], tp[-1])
            if idx.size < 2:
                continue
            for k, h in enumerate(hs):
                if ts + h > tp[-1] + 1e-9:
                    break
                j = idx[np.argmin(np.abs(R.rec.t[idx] - (ts + h)))]
                if abs(R.rec.t[j] - (ts + h)) > 0.01:
                    continue
                pm = np.array([np.interp(R.rec.t[j], tp, sp[:, 0]), np.interp(R.rec.t[j], tp, sp[:, 1])])
                errs[k].append(1e3 * np.linalg.norm(pm - R.rec.p[j]))
    rms = np.array([np.sqrt(np.mean(np.square(e))) if e else np.nan for e in errs])
    p90 = np.array([np.percentile(e, 90) if e else np.nan for e in errs])
    return hs, rms, p90


# ------------------------------------------------------------------------------------- entry
def run(rec, seg_yaml, res, cfg, save_dir=None, segment_id=None, horizon=None, every=None):
    horizon = horizon or cfg["plot"]["horizon"]
    every = every or cfg["plot"]["every"]
    R = Results(rec, seg_yaml, res, cfg)
    if save_dir:
        import matplotlib
        matplotlib.use("Agg")
        os.makedirs(save_dir, exist_ok=True)
        fig = summary(R, horizon)
        fig.savefig(os.path.join(save_dir, "summary.png"), dpi=100)
        plt.close(fig)
        page = 10.0
        for k, t0 in enumerate(np.arange(rec.t[0], rec.t[-1], page)):
            fig = timeline(R, horizon, every, (t0, t0 + page))
            fig.savefig(os.path.join(save_dir, f"timeline_{k:03d}.png"), dpi=90)
            plt.close(fig)
        view = SegmentView(R, None, horizon, every)
        view.fan = True
        for i, g in enumerate(R.segments):
            if segment_id is not None and g["id"] != segment_id:
                continue
            view.i = i
            view.draw()
            view.fig.savefig(os.path.join(save_dir, f"segment_{g['id']:03d}_{g['label']}.png"), dpi=80)
        plt.close(view.fig)
        print(f"wrote plots to {save_dir}")
        return
    view = SegmentView(R, segment_id, horizon, every)
    view.draw()
    timeline(R, horizon, every)
    summary(R, horizon)
    plt.show()
