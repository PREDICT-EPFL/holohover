"""Window to check and correct the segments (matplotlib).

Left: speed, position and yaw over time with the segments (green: glide, orange: wall
impact, red: hit) and the detected events (blue: wall impact, red: hit, grey: flight, an
acceleration spike without velocity change). Hit segments are kept but not fitted. Right: the table
with the walls and the selected segment. The time slider at the bottom shows where the puck
is at that time (red dot on the table, red line on the time plots).

    drag on a time plot      new segment with the current label
                             (dragging over the selected segment changes its start and end)
    click on a time plot     move the time slider there
    time slider              red dot on the table at that time, selects the segment there
                             (when zoomed in, the time plots follow it with the same zoom)
    g / w / h                current label glide / wall_impact / hit (also relabels the selected segment)
    d, delete                delete the selected segment
    n / b                    next / previous segment (zooms to it)
    a                        show the whole recording
    u                        undo
    s                        save segments.yaml
    q                        quit (press twice if there are unsaved changes)
The toolbar zoom and pan work as usual (no segment editing while one of them is active).
"""
import copy

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider, SpanSelector

import data
import model

COLORS = {"glide": "tab:green", "wall_impact": "tab:orange", "hit": "tab:red"}
LABEL_KEYS = {"g": "glide", "w": "wall_impact", "h": "hit"}
EVENT_STYLE = {"wall": ("tab:blue", ":"), "hit": ("tab:red", "-"), "flight": ("0.6", ":")}
KEYS = ["g", "w", "h", "d", "delete", "n", "b", "a", "u", "s", "q", "backspace"]


def _free_keys():
    for k, v in matplotlib.rcParams.items():
        if k.startswith("keymap.") and isinstance(v, list):
            matplotlib.rcParams[k] = [key for key in v if key not in KEYS]


class Review:
    def __init__(self, rec, seg_file, cfg):
        self.rec, self.seg_file, self.cfg = rec, seg_file, cfg
        self.doc = data.load_segments(seg_file, rec)
        self.walls = model.walls_from_yaml(self.doc.get("walls", []))
        self.events = self.doc.get("events", [])
        self.segments = [dict(g) for g in self.doc.get("segments", [])]
        self.label = "glide"
        self.selected = None
        self.history = []
        self.dirty = False
        self.quit_armed = False
        self.spans = []

        _free_keys()
        self.fig = plt.figure(figsize=(16, 9))
        self.fig.canvas.manager.set_window_title(f"segments - {seg_file}")
        gs = self.fig.add_gridspec(3, 2, width_ratios=[3, 1], hspace=0.08, wspace=0.12,
                                   left=0.05, right=0.98, top=0.95, bottom=0.14)
        self.ax_s = self.fig.add_subplot(gs[0, 0])
        self.ax_p = self.fig.add_subplot(gs[1, 0], sharex=self.ax_s)
        self.ax_y = self.fig.add_subplot(gs[2, 0], sharex=self.ax_s)
        self.ax_xy = self.fig.add_subplot(gs[:, 1])
        self.status = self.fig.text(0.05, 0.015, "", fontsize=10, family="monospace")
        self._draw_static()
        self._init_cursor()
        self._draw_segments()
        self.selectors = [SpanSelector(ax, self.on_span, "horizontal", useblit=True, minspan=0.02,
                                       props=dict(alpha=0.3, facecolor="tab:purple"))
                          for ax in (self.ax_s, self.ax_p, self.ax_y)]
        self.fig.canvas.mpl_connect("button_press_event", self.on_click)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        self._update_status("ready")

    # time cursor
    def _init_cursor(self):
        r = self.rec
        left = self.ax_y.get_position()
        ax_slider = self.fig.add_axes([left.x0, 0.065, left.width, 0.025])
        self.slider = Slider(ax_slider, "", float(r.t[0]), float(r.t[-1]), valinit=float(r.t[0]), color="tab:red")
        self.slider.valtext.set_text("")
        self.cursor_lines = [ax.axvline(r.t[0], color="tab:red", lw=1.2) for ax in (self.ax_s, self.ax_p, self.ax_y)]
        self.cursor_trail, = self.ax_xy.plot([], [], color="tab:red", lw=1, alpha=0.5, zorder=4)
        self.cursor_dot, = self.ax_xy.plot([], [], "o", color="tab:red", ms=9, mec="k", zorder=5)
        self.slider.on_changed(self.on_time)
        self._show_time(self.slider.val)

    def on_time(self, t):
        self._follow(t)
        self._show_time(t)
        self._select_at(t)

    def _follow(self, t, edge=0.05):
        """Pan the time plots (same zoom) when t gets within `edge` of the view border."""
        lo, hi = self.ax_s.get_xlim()
        m = edge * (hi - lo)
        if lo + m <= t <= hi - m:
            return
        shift = t - (hi - m) if t > hi - m else t - (lo + m)
        t0, t1 = float(self.rec.t[0]), float(self.rec.t[-1])
        shift = float(np.clip(shift, t0 - lo, t1 - hi)) if hi - lo < t1 - t0 else 0.0
        if shift:
            self.ax_s.set_xlim(lo + shift, hi + shift)

    def _select_at(self, t):
        """Select the segment at time t (the selected one wins where two segments touch)."""
        sel = self.selected
        if sel is None or not self.segments[sel]["t0"] <= t <= self.segments[sel]["t1"]:
            sel = next((i for i, g in enumerate(self.segments) if g["t0"] <= t <= g["t1"]), None)
        if sel == self.selected:
            return
        self.selected = sel
        self._draw_segments()
        warn = self._check(self.segments[sel]) if sel is not None else ""
        self._update_status(f"WARNING: {warn}" if warn else "")

    def _show_time(self, t):
        """Puck position at time t on the table (with the last 0.5 s as a trail)."""
        r = self.rec
        i = int(np.clip(np.searchsorted(r.t, t), 0, r.t.size - 1))
        trail = r.window(r.t[i] - 0.5, r.t[i])
        self.cursor_dot.set_data([r.p[i, 0]], [r.p[i, 1]])
        self.cursor_trail.set_data(r.p[trail, 0], r.p[trail, 1])
        for line in self.cursor_lines:
            line.set_xdata([r.t[i], r.t[i]])
        seg = next((g for g in self.segments if g["t0"] <= r.t[i] <= g["t1"]), None)
        where = f"{seg['label']}" if seg else "not in a segment"
        self.slider.valtext.set_text(f"{r.t[i]:.3f} s")
        self.ax_xy.set_title(f"table frame   t = {r.t[i]:.2f} s   x = {r.p[i, 0]:.3f}, y = {r.p[i, 1]:.3f} m\n"
                             f"speed {r.speed[i]:.2f} m/s, {where}", fontsize=9)
        self.fig.canvas.draw_idle()

    # drawing
    def _draw_static(self):
        r = self.rec
        self.ax_s.plot(r.t, r.speed, lw=0.7, color="k")
        self.ax_s.set_ylabel("speed [m/s]")
        self.ax_s.set_ylim(0, min(3.0, np.nanpercentile(r.speed, 99.5) * 1.3))
        self.ax_p.plot(r.t, r.p[:, 0], lw=0.7, label="x")
        self.ax_p.plot(r.t, r.p[:, 1], lw=0.7, label="y")
        self.ax_p.set_ylabel("position [m]")
        self.ax_p.legend(loc="upper right", fontsize=8)
        ok = r.yaw_ok
        self.ax_y.plot(r.t[ok], r.yaw[ok], ".", ms=1, color="tab:purple", label="yaw")
        self.ax_y.plot(r.t[~ok], r.yaw[~ok], "x", ms=3, color="tab:red", label="invalid")
        self.ax_y.set_ylabel("yaw [rad]")
        self.ax_y.set_xlabel("time [s]")
        self.ax_y.legend(loc="upper right", fontsize=8)
        for a, b in zip(r.chunks[:-1], r.chunks[1:]):
            for ax in (self.ax_s, self.ax_p, self.ax_y):
                ax.axvspan(r.t[a[1] - 1], r.t[b[0]], color="0.3", alpha=0.4, lw=0)
        for e in self.events:
            color, ls = EVENT_STYLE.get(e["kind"], ("k", "-"))
            for ax in (self.ax_s, self.ax_p, self.ax_y):
                ax.axvline(e["t"], color=color, ls=ls, lw=0.8, alpha=0.8)
        for ax in (self.ax_s, self.ax_p):
            plt.setp(ax.get_xticklabels(), visible=False)
        self.ax_s.set_xlim(r.t[0], r.t[-1])  # aligned with the time slider below
        self.ax_xy.plot(r.p[:, 0], r.p[:, 1], lw=0.4, color="0.75")
        self._draw_walls(self.ax_xy, r.p)
        contacts = np.array([e["contact"] for e in self.events if e["kind"] == "wall" and "contact" in e])
        if contacts.size:
            self.ax_xy.plot(contacts[:, 0], contacts[:, 1], ".", color="tab:blue", ms=4)
        self.sel_line, = self.ax_xy.plot([], [], lw=2)
        self.ax_xy.set_aspect("equal")
        self.ax_xy.set_xlabel("x [m]")
        self.ax_xy.set_ylabel("y [m]")

    def _draw_walls(self, ax, p):
        lo, hi = p.min(0) - 0.05, p.max(0) + 0.05
        for w in self.walls:
            n, d = w["normal"], w["offset"]
            t = np.array([-n[1], n[0]])
            s = np.array([lo @ t, hi @ t, np.array([lo[0], hi[1]]) @ t, np.array([hi[0], lo[1]]) @ t])
            pts = np.outer([s.min(), s.max()], t) + d * n
            ax.plot(pts[:, 0], pts[:, 1], color="tab:blue", lw=1.5)
            ax.annotate(w["name"], pts.mean(0), color="tab:blue", fontsize=8)

    def _draw_segments(self):
        for patch in self.spans:
            patch.remove()
        self.spans = []
        for i, g in enumerate(self.segments):
            sel = i == self.selected
            for ax in (self.ax_s, self.ax_p, self.ax_y):
                self.spans.append(ax.axvspan(g["t0"], g["t1"], color=COLORS.get(g["label"], "0.5"),
                                             alpha=0.45 if sel else 0.2, lw=2 if sel else 0,
                                             ec="k" if sel else None))
        if self.selected is not None:
            g = self.segments[self.selected]
            idx = self.rec.window(g["t0"], g["t1"])
            self.sel_line.set_data(self.rec.p[idx, 0], self.rec.p[idx, 1])
            self.sel_line.set_color(COLORS.get(g["label"], "k"))
        else:
            self.sel_line.set_data([], [])
        n = {k: sum(g["label"] == k for g in self.segments) for k in COLORS}
        self.ax_s.set_title(f"{n['glide']} glides, {n['wall_impact']} wall impacts, {n['hit']} hits, "
                            f"{sum(g['t1'] - g['t0'] for g in self.segments):.1f} s"
                            f"{'   (unsaved)' if self.dirty else ''}", fontsize=10)
        self.fig.canvas.draw_idle()

    def _update_status(self, msg):
        sel = ""
        if self.selected is not None:
            g = self.segments[self.selected]
            sel = f"selected: {g['label']} {g['t0']:.3f}-{g['t1']:.3f} s ({g['t1'] - g['t0']:.2f} s)"
        self.status.set_text(f"label for new segments: {self.label:12s} {sel}\n{msg}    "
                             "[drag] new  [click] select  [slider] puck on the table  g/w/h label  d delete  "
                             "n/b next/prev  a all  u undo  s save  q quit")
        self.fig.canvas.draw_idle()

    # editing
    def _push(self):
        self.history.append((copy.deepcopy(self.segments), self.selected))
        self.dirty = True
        self.quit_armed = False

    def _check(self, g):
        """Warnings for a segment: what it contains that the model does not explain."""
        inside = [e for e in self.events if g["t0"] < e["t"] < g["t1"]]
        n_wall = sum(e["kind"] == "wall" for e in inside)
        n_hit = sum(e["kind"] == "hit" for e in inside)
        msgs = []
        if g["label"] == "hit":
            if n_hit != 1:
                msgs.append(f"hit segment with {n_hit} detected hits")
        elif n_hit:
            msgs.append("contains a hit (the model cannot explain it, label it hit)")
        if g["label"] == "wall_impact" and n_wall != 1:
            msgs.append(f"wall_impact with {n_wall} detected wall impacts")
        if g["label"] == "glide" and n_wall:
            msgs.append("glide contains a wall impact (fine for the fit, but label it wall_impact)")
        for a, b in zip(self.rec.chunks[:-1], self.rec.chunks[1:]):
            if g["t0"] < self.rec.t[b[0]] and self.rec.t[a[1] - 1] < g["t1"]:
                msgs.append("spans a gap in the data")
                break
        return "; ".join(msgs)

    def _busy(self):
        tb = getattr(self.fig.canvas.manager, "toolbar", None)
        return tb is not None and str(getattr(tb, "mode", "")) != ""

    def on_span(self, t0, t1):
        if self._busy() or t1 - t0 < 0.02:
            return
        t0, t1 = float(t0), float(t1)
        sel = self.segments[self.selected] if self.selected is not None else None
        if sel is not None and t0 < sel["t1"] and sel["t0"] < t1:
            self._push()
            sel["t0"], sel["t1"] = round(t0, 4), round(t1, 4)
            msg = "changed the selected segment"
            g = sel
        else:
            if any(t0 < g["t1"] and g["t0"] < t1 for g in self.segments):
                self._update_status("overlaps an existing segment: select it to change it, or delete it first")
                return
            self._push()
            g = {"id": None, "label": self.label, "t0": round(t0, 4), "t1": round(t1, 4)}
            self.segments.append(g)
            msg = f"new {self.label}"
        self.segments.sort(key=lambda s: s["t0"])
        self.selected = self.segments.index(g)
        warn = self._check(g)
        self._draw_segments()
        self._update_status(msg + (f"  WARNING: {warn}" if warn else ""))

    def on_click(self, event):
        if self._busy() or event.inaxes not in (self.ax_s, self.ax_p, self.ax_y) or event.button != 1:
            return
        # the slider selects the segment there
        self.slider.set_val(float(np.clip(event.xdata, self.slider.valmin, self.slider.valmax)))

    def _goto(self, i):
        if not self.segments:
            return
        self.selected = i % len(self.segments)
        g = self.segments[self.selected]
        pad = max(0.5, 0.5 * (g["t1"] - g["t0"]))
        self.ax_s.set_xlim(g["t0"] - pad, g["t1"] + pad)
        self.slider.set_val(float(g["t0"]))  # keeps this selection: it contains t0
        self._draw_segments()
        warn = self._check(g)
        self._update_status(f"segment {self.selected + 1}/{len(self.segments)}" + (f"  WARNING: {warn}" if warn else ""))

    def on_key(self, event):
        k = event.key
        if k in LABEL_KEYS:
            self.label = LABEL_KEYS[k]
            if self.selected is not None and self.segments[self.selected]["label"] != self.label:
                self._push()
                self.segments[self.selected]["label"] = self.label
                self._draw_segments()
            self._update_status(f"label: {self.label}")
        elif k in ("d", "delete", "backspace") and self.selected is not None:
            self._push()
            del self.segments[self.selected]
            self.selected = None
            self._draw_segments()
            self._update_status("deleted")
        elif k == "n":
            self._goto(0 if self.selected is None else self.selected + 1)
        elif k == "b":
            self._goto(-1 if self.selected is None else self.selected - 1)
        elif k == "a":
            self.ax_s.set_xlim(self.rec.t[0], self.rec.t[-1])
            self.fig.canvas.draw_idle()
        elif k == "u":
            if self.history:
                self.segments, self.selected = self.history.pop()
                self.dirty = True
                self._draw_segments()
                self._update_status("undone")
        elif k == "s":
            self.save()
        elif k == "q":
            if self.dirty and not self.quit_armed:
                self.quit_armed = True
                self._update_status("unsaved changes: press q again to quit without saving, s to save")
            else:
                plt.close(self.fig)

    def save(self):
        self.segments.sort(key=lambda s: s["t0"])
        for i, g in enumerate(self.segments):
            g["id"] = i
        self.doc["segments"] = self.segments
        data.save_yaml(self.seg_file, self.doc,
                       header="Puck identification segments, times in s from t_ref (first puck stamp).\n"
                              "Edit with `identify.py review`. Labels: glide, wall_impact, hit (not fitted).")
        self.dirty = False
        self._draw_segments()
        self._update_status(f"saved {self.seg_file}")
        print(f"saved {len(self.segments)} segments to {self.seg_file}")


def run(rec, seg_file, cfg):
    review = Review(rec, seg_file, cfg)  # keep a reference: matplotlib holds the callbacks weakly
    plt.show()
    return review
