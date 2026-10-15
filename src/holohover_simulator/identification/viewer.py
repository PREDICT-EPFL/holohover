"""Window comparing the measured data with the model (Dear ImGui / ImPlot), three tabs.

  segments  one segment at a time: measured (dots) and the model simulated from the fitted
            initial state (red) for x, y, vx, vy, yaw, yaw rate, the position and yaw errors,
            and the path on the table. Optional: predictions (blue) started every `every`
            seconds from the measured state, i.e. what an online predictor (the MPC) would get.
  timeline  ground truth vs model over the whole recording: the fitted model (red, every segment
            from its fitted initial state) and the predictions over `horizon` from the measured
            state (blue), their errors, segments, hits and wall impacts. The slider moves the view.
  summary   impact law against every impact, glide accelerations, spin decay, prediction error
            against the horizon and fit error per segment.

    mouse wheel / right drag   zoom (time plots stay aligned), left drag: pan, double click: fit
    click a legend entry       hide / show that series
    n / b                      next / previous segment
    h                          predictions from the measured state on / off
"""
import numpy as np
from imgui_bundle import imgui, implot

import gui
import model
import plot

Y_FIT = implot.AxisFlags_.auto_fit | implot.AxisFlags_.range_fit  # y follows the visible time range
TIMELINE_WINDOW = 10.0  # [s] width of the timeline view when it opens


class Viewer:
    SIDEBAR = 330

    def __init__(self, R, start, horizon, every):
        self.R, self.horizon, self.every = R, horizon, every
        ids = [g["id"] for g in R.segments]
        self.i = ids.index(start) if start in ids else 0
        self.fan = False
        self.scroll_list = True
        self._curves = {}
        self._fans = {}
        self._timeline = None
        self._summary = None
        self.seg_axis = None
        self._set_segment(self.i)
        r = R.rec
        self.tl_axis = gui.TimeAxis(r.t[0], r.t[-1], TIMELINE_WINDOW)
        self.title = "puck model: " + ", ".join(f"{k} = {v:.4g}" for k, v in R.p.items())
        self.yaw_ok = float(r.yaw_ok.mean())
        self.warning = ""
        if self.yaw_ok < 0.8:
            self.warning = (f"the yaw is valid in only {100 * self.yaw_ok:.0f}% of the samples (mocap orientation "
                            "lost or flipping, EKF yaw rate exploding): predictions need a valid yaw at their start, "
                            "so most are missing. Check the mocap rigid body / EKF of this recording.")
            print("warning:", self.warning)

    # ------------------------------------------------------------------ data (cached)
    def _set_segment(self, i):
        self.i = i % len(self.R.segments)
        c = self.curves()
        r = self.R.rec
        self.seg_axis = gui.TimeAxis(r.t[c["ctx"][0]], r.t[c["ctx"][-1]])
        self.seg_axis.show_all()
        self.scroll_list = True

    def curves(self):
        if self.i not in self._curves:
            self._curves[self.i] = plot.segment_curves(self.R, self.R.segments[self.i])
        return self._curves[self.i]

    def fan_curves(self):
        """Predictions from the measured state around the segment, joined per state column."""
        if self.i not in self._fans:
            g = self.R.segments[self.i]
            pad = plot.SEGMENT_PAD
            preds = self.R.predictions(g["t0"] - pad, g["t1"] + pad, self.horizon, self.every)
            self._fans[self.i] = {
                "t": {c: gui.nan_joined([(tp, sp[:, c]) for _, tp, sp in preds]) for c in (0, 1, 3, 4, 5)},
                "xy": gui.nan_joined([(sp[:, 0], sp[:, 1]) for _, _, sp in preds]),
                "start": np.array([sp[0, 0:2] for _, _, sp in preds]).reshape(-1, 2)}
        return self._fans[self.i]

    def timeline_data(self):
        if self._timeline is None:
            R, r = self.R, self.R.rec
            print("computing the predictions of the timeline ...")
            preds = R.predictions(r.t[0], r.t[-1], self.horizon, self.every, inside_runs=False)
            err_t, err = [], []
            for ts, tp, sp in preds:
                idx = r.window(tp[0], tp[-1])
                if idx.size:
                    j = idx[-1]
                    pm = [np.interp(r.t[j], tp, sp[:, 0]), np.interp(r.t[j], tp, sp[:, 1])]
                    err_t.append(ts)
                    err.append(1e3 * np.linalg.norm(np.array(pm) - r.p[j]))
            # the fitted model: every segment simulated from its fitted initial state
            fits = []
            for i in range(len(R.segments)):
                if i not in self._curves:
                    self._curves[i] = plot.segment_curves(R, R.segments[i])
                fits.append(self._curves[i])
            self._timeline = {
                "fit_x": gui.nan_joined([(c["t"], c["st"][:, 0]) for c in fits]),
                "fit_y": gui.nan_joined([(c["t"], c["st"][:, 1]) for c in fits]),
                "fit_err": gui.nan_joined([(r.t[c["idx"]], c["err_mm"]) for c in fits]),
                "x": gui.nan_joined([(tp, sp[:, 0]) for _, tp, sp in preds]),
                "y": gui.nan_joined([(tp, sp[:, 1]) for _, tp, sp in preds]),
                "start_t": np.array([tp[0] for _, tp, _ in preds]),
                "start": np.array([sp[0, 0:2] for _, _, sp in preds]).reshape(-1, 2),
                "err": (np.array(err_t), np.array(err)),
                "bands": [(g["t0"], g["t1"], gui.LABEL_COLORS.get(g["label"], gui.MEAS), 0.15) for g in R.segments],
                "hit": [e["t"] for e in R.events if e["kind"] == "hit"],
                "wall": [e["t"] for e in R.events if e["kind"] == "wall"]}
        return self._timeline

    def summary_data(self):
        if self._summary is None:
            R = self.R
            print("computing the summary ...")
            hs, rms, p90 = plot.prediction_error(R, self.horizon)
            self._summary = {"impacts": plot.impact_stats(R), "glides": plot.glide_stats(R),
                             "horizon": (hs, rms, p90)}
        return self._summary

    # ------------------------------------------------------------------ window
    def gui(self):
        with gui.fullscreen("viewer"):
            if not imgui.is_any_item_active():
                if gui.key(imgui.Key.n):
                    self._set_segment(self.i + 1)
                if gui.key(imgui.Key.b):
                    self._set_segment(self.i - 1)
                if gui.key(imgui.Key.h):
                    self.fan = not self.fan
            imgui.text(self.title)
            if self.warning:
                imgui.text_colored(gui.vec4(gui.WALL), "warning: " + self.warning)
            if imgui.begin_tab_bar("views"):
                for name, draw in (("segments", self.segments_tab), ("timeline", self.timeline_tab),
                                   ("summary", self.summary_tab)):
                    if imgui.begin_tab_item(name)[0]:
                        draw()
                        imgui.end_tab_item()
                imgui.end_tab_bar()

    # ------------------------------------------------------------------ segments
    def segments_tab(self):
        imgui.begin_child("sidebar", imgui.ImVec2(self.SIDEBAR, 0))
        self.segment_sidebar()
        imgui.end_child()
        imgui.same_line()
        avail = imgui.get_content_region_avail()
        imgui.begin_child("time", imgui.ImVec2(avail.x * 0.66, 0))
        self.segment_plots()
        imgui.end_child()
        imgui.same_line()
        imgui.begin_child("table", imgui.ImVec2(0, 0))
        self.segment_table_plot()
        imgui.end_child()

    def segment_sidebar(self):
        R = self.R
        g = R.segments[self.i]
        c = self.curves()
        imgui.text_colored(gui.vec4(gui.LABEL_COLORS.get(g["label"], gui.MEAS)),
                           f"segment {g['id']} ({self.i + 1}/{len(R.segments)}): {g['label']}")
        imgui.text(f"{g['t0']:.2f} - {g['t1']:.2f} s ({g['t1'] - g['t0']:.2f} s)")
        imgui.text(f"rms position {g['rms_position_mm']:.2f} mm, yaw {g['rms_yaw_deg']:.2f} deg")
        imgui.text(f"model impacts: {len(c['impacts'])}")
        bw = imgui.ImVec2((self.SIDEBAR - imgui.get_style().item_spacing.x) / 2, 0)
        if imgui.button("< prev (b)", bw):
            self._set_segment(self.i - 1)
        imgui.same_line()
        if imgui.button("next (n) >", bw):
            self._set_segment(self.i + 1)
        self.fan = imgui.checkbox("predictions from the measured state (h)", self.fan)[1]
        if self.fan:
            imgui.text_disabled(f"over {self.horizon:.2f} s, started every {self.every:.2f} s")
        imgui.separator()
        flags = imgui.TableFlags_.scroll_y | imgui.TableFlags_.row_bg | imgui.TableFlags_.sizing_stretch_prop
        if imgui.begin_table("segments", 3, flags, imgui.ImVec2(0, 0)):
            imgui.table_setup_scroll_freeze(0, 1)
            for name in ("id", "label", "rms [mm]"):
                imgui.table_setup_column(name)
            imgui.table_headers_row()
            for i, s in enumerate(R.segments):
                imgui.table_next_row()
                imgui.table_next_column()
                if imgui.selectable(f"{s['id']}##row", i == self.i, imgui.SelectableFlags_.span_all_columns)[0]:
                    self._set_segment(i)
                    self.scroll_list = False
                if i == self.i and self.scroll_list:
                    imgui.set_scroll_here_y()
                    self.scroll_list = False
                imgui.table_next_column()
                imgui.text_colored(gui.vec4(gui.LABEL_COLORS.get(s["label"], gui.MEAS)), s["label"])
                imgui.table_next_column()
                imgui.text(f"{s['rms_position_mm']:.2f}")
            imgui.end_table()

    def segment_plots(self):
        r = self.R.rec
        c = self.curves()
        g = self.R.segments[self.i]
        ctx, idx, st = c["ctx"], c["idx"], c["st"]
        fan = self.fan_curves() if self.fan else None
        vlab = "(EKF)" if r.measured_velocity else "(smoothed)"
        yaw_ok = ctx[r.yaw_ok[ctx]]
        yaw_err = np.degrees(plot._wrap(c["st_meas"][:, 2] - r.yaw[idx]))
        ok = r.yaw_ok[idx]
        rows = [
            (("x", "x [m]", r.p[ctx, 0], ctx, 0), ("y", "y [m]", r.p[ctx, 1], ctx, 1)),
            (("vx", f"vx [m/s] {vlab}", r.v[ctx, 0], ctx, 3), ("vy", f"vy [m/s] {vlab}", r.v[ctx, 1], ctx, 4)),
            (("yaw", "yaw [rad] (unwrapped)", r.yaw_u[yaw_ok], yaw_ok, 2),
             ("w", f"yaw rate [rad/s] {vlab}", r.w[yaw_ok], yaw_ok, 5)),
        ]
        h = gui.plot_heights(4)
        w = (imgui.get_content_region_avail().x - imgui.get_style().item_spacing.x) / 2
        for row in rows:
            for k, (name, ylabel, y, sl, col) in enumerate(row):
                if k:
                    imgui.same_line()
                if self._begin_time_plot(name, ylabel, w, h, g, legend=name == "x"):
                    gui.dots("measured", r.t[sl], y, gui.MEAS, 1.5)
                    if fan is not None and col != 2:
                        gui.line("predictions", *fan["t"][col], gui.PRED, 1.0)
                    gui.line("fitted model", c["t"], c["yaw"] if col == 2 else st[:, col], gui.MODEL, 2.5)  # on top
                    implot.end_plot()
        if self._begin_time_plot("err", "position error [mm]", w, h, g, x_label="time [s]"):
            gui.line("position error", r.t[idx], c["err_mm"], gui.MODEL, 1.5, legend=False)
            implot.end_plot()
        imgui.same_line()
        if self._begin_time_plot("yaw_err", "yaw error [deg]", w, h, g, x_label="time [s]"):
            gui.dots("yaw error", r.t[idx][ok], yaw_err[ok], gui.MODEL, 1.5, legend=False)
            implot.end_plot()

    def _begin_time_plot(self, name, ylabel, w, h, g, x_label=None, legend=False):
        flags = implot.Flags_.no_menus | (0 if legend else implot.Flags_.no_legend)
        if not implot.begin_plot(f"##{name}", imgui.ImVec2(w, h), flags):
            return False
        self.seg_axis.setup()
        implot.setup_axis(gui.X1, x_label, 0 if x_label else implot.AxisFlags_.no_tick_labels)
        implot.setup_axis(gui.Y1, ylabel, Y_FIT)
        implot.setup_legend(implot.Location_.north_west)
        gui.spans([(g["t0"], g["t1"], (1.0, 1.0, 1.0, 1.0), 0.07)])
        gui.vlines("model impact", [g["t0"] + t for t, _ in self.curves()["impacts"]], gui.MODEL, 1.0, legend=False)
        return True

    def segment_table_plot(self):
        r = self.R.rec
        c = self.curves()
        st, ctx = c["st"], c["ctx"]
        if not implot.begin_plot("##path", imgui.ImVec2(-1, -1), implot.Flags_.equal | implot.Flags_.no_menus):
            return
        implot.setup_axes("x [m]", "y [m]", implot.AxisFlags_.auto_fit, implot.AxisFlags_.auto_fit)
        implot.setup_legend(implot.Location_.north_west)
        gui.dots("measured", r.p[ctx, 0], r.p[ctx, 1], gui.MEAS, 1.5)
        if self.fan:
            f = self.fan_curves()
            gui.line("predictions", *f["xy"], gui.PRED, 1.0)
            gui.dots("prediction start", f["start"][:, 0], f["start"][:, 1], gui.PRED, 2.0, legend=False)
        gui.line("fitted model", st[:, 0], st[:, 1], gui.MODEL, 2.5)  # on top
        gui.dots("model start", st[:1, 0], st[:1, 1], gui.MODEL, 5.0, legend=False)
        pts = np.vstack([r.p[ctx], st[:, 0:2]])
        lo, hi = pts.min(0) - 0.05, pts.max(0) + 0.05
        for w in self.R.walls:
            n, d = np.asarray(w["normal"]), w["offset"]
            if np.min(np.abs(pts @ n - d)) > 0.2:  # only the walls near the shown area
                continue
            t = np.array([-n[1], n[0]])
            cs = np.array([lo, hi, [lo[0], hi[1]], [hi[0], lo[1]]]) @ t
            a = np.outer([cs.min(), cs.max()], t) + d * n
            gui.line("walls", a[:, 0], a[:, 1], gui.WALL_LINE, 2.0)
        implot.end_plot()

    # ------------------------------------------------------------------ timeline
    def timeline_tab(self):
        d = self.timeline_data()
        r, ax = self.R.rec, self.tl_axis
        imgui.text("Ground truth vs model")
        imgui.same_line()
        imgui.text_disabled(f"  red: fitted model, each segment simulated from its fitted initial state;  blue: "
                            f"predictions over {self.horizon:.2f} s started every {self.every:.2f} s from the measured "
                            "state;  click a legend entry to hide it")
        slider_h = imgui.get_frame_height_with_spacing()
        h = gui.plot_heights(3, slider_h)
        for name, ylabel, col in (("x", "x [m]", 0), ("y", "y [m]", 1), ("err", "position error [mm]", None)):
            legend = implot.Flags_.no_legend if name == "y" else 0  # y has the same items as x
            if not implot.begin_plot(f"##tl_{name}", imgui.ImVec2(-1, h), implot.Flags_.no_menus | legend):
                continue
            ax.setup()
            last = col is None
            implot.setup_axis(gui.X1, "time [s]" if last else None, 0 if last else implot.AxisFlags_.no_tick_labels)
            implot.setup_axis(gui.Y1, ylabel, Y_FIT)
            if last:
                implot.setup_axis_scale(gui.Y1, implot.Scale_.log10)
            implot.setup_legend(implot.Location_.north_east, implot.LegendFlags_.horizontal)
            gui.spans(d["bands"])
            if last:
                gui.dots("prediction error at its end", *d["err"], gui.PRED, 2.0)
                gui.line("fitted model error", *d["fit_err"], gui.MODEL, 1.5)
            else:
                gui.line("prediction from the measured state", *d[name], gui.PRED, 1.0)
                gui.dots("prediction start", d["start_t"], d["start"][:, col], gui.PRED, 2.0)
                gui.dots("measured (ground truth)", r.t, r.p[:, col], gui.MEAS, 1.5)
                gui.line("fitted model (per segment)", *d["fit_" + name], gui.MODEL, 2.0)  # on top
            gui.vlines("hit (not modelled)", d["hit"], gui.HIT, 1.5)
            gui.vlines("wall impact", d["wall"], gui.FLIGHT, 1.0)
            implot.end_plot()
        lo, hi = ax.lo.value, ax.hi.value
        w = hi - lo
        imgui.set_next_item_width(-1)
        changed, v = imgui.slider_float("##view", lo, ax.t_min, max(ax.t_min, ax.t_max - w), "view from %.1f s")
        if changed:
            ax.show(v, v + w)

    # ------------------------------------------------------------------ summary
    def summary_tab(self):
        d = self.summary_data()
        p, R = self.R.p, self.R
        avail = imgui.get_content_region_avail()
        sp = imgui.get_style().item_spacing
        size = imgui.ImVec2((avail.x - 2 * sp.x) / 3, (avail.y - sp.y) / 2)
        imp, gl = d["impacts"], d["glides"]

        def begin(title, xlabel, ylabel):
            if not implot.begin_plot(title, size, implot.Flags_.no_menus):
                return False
            implot.setup_axes(xlabel, ylabel, implot.AxisFlags_.auto_fit, implot.AxisFlags_.auto_fit)
            implot.setup_legend(implot.Location_.north_east)
            return True

        if begin("normal restitution, one marker per impact", "contact position along the wall [m]", "-v_n' / v_n"):
            for name in sorted({i["wall"] for i in imp}):
                sub = [i for i in imp if i["wall"] == name]
                gui.scatter(name, [i["along"] for i in sub], [i["e_n"] for i in sub])
            gui.hline(f"model e_n = {p['e_n']:.3f}", p["e_n"], gui.MODEL, 1.5)
            implot.end_plot()
        imgui.same_line()
        if begin("tangential: stick (left, -e_t) to slide (right)", "|u_t| / |v_n| before (u_t = v_t - r w)",
                 "u_t' / u_t"):
            psi = np.array([i["psi"] for i in imp])
            ratio = np.array([i["ratio"] for i in imp])
            ok = np.isfinite(ratio)
            gui.dots("measured", psi[ok], ratio[ok], gui.MEAS, 3.0)
            xs = np.linspace(1e-3, max(1.0, float(np.nanmax(psi)) * 1.1 if psi.size else 1.0), 300)
            gui.line(f"model mu = {p['mu']:.3f}, e_t = {p['e_t']:.3f}", xs,
                     model.tangential_ratio(xs, p["e_n"], p["mu"], p["e_t"], R.kappa), gui.MODEL, 1.5)
            gui.hline("-e_t", -p["e_t"], gui.FLIGHT)
            implot.end_plot()
        imgui.same_line()
        if begin(f"glide acceleration: c_v = {p['c_v']:.4f} 1/s, g = ({1e3 * p['g_x']:.1f}, {1e3 * p['g_y']:.1f}) mm/s^2",
                 "model  g - c_v v  [mm/s^2]", "measured (quadratic fit per glide) [mm/s^2]"):
            if gl:
                g = np.array([p["g_x"], p["g_y"]])
                meas = 1e3 * np.array([row["a"] for row in gl])
                pred = 1e3 * np.array([g - p["c_v"] * row["v"] for row in gl])
                gui.scatter("a_x", pred[:, 0], meas[:, 0])
                gui.scatter("a_y", pred[:, 1], meas[:, 1])
                lim = float(np.nanmax(np.abs(np.concatenate([meas.ravel(), pred.ravel()])))) * 1.1
                gui.line("measured = model", [-lim, lim], [-lim, lim], gui.FLIGHT)
            implot.end_plot()
        if begin("spin decay", "yaw rate [rad/s]", "yaw acceleration [rad/s^2]"):
            if gl:
                w = np.array([row["w"] for row in gl])
                dw = np.array([row["dw"] for row in gl])
                ok = np.isfinite(w) & np.isfinite(dw)
                gui.dots("measured (per glide)", w[ok], dw[ok], gui.MEAS, 3.0)
                if ok.any():
                    ws = np.linspace(w[ok].min() - 1, w[ok].max() + 1, 200)
                    gui.line(f"model c_w = {p['c_w']:.3f}, c_wc = {p['c_wc']:.3f}", ws,
                             -p["c_w"] * ws - p["c_wc"] * np.sign(ws), gui.MODEL, 1.5)
            implot.end_plot()
        imgui.same_line()
        if begin("prediction from the measured state, inside the segments", "prediction horizon [s]",
                 "position error [mm]"):
            hs, rms, p90 = d["horizon"]
            gui.line("rms", hs, rms, gui.PRED, 2.0)
            gui.line("90th percentile", hs, p90, gui.WALL, 1.0)
            implot.end_plot()
        imgui.same_line()
        if begin("fit error per segment", "segment", "rms position error [mm]"):
            rms = np.array([g["rms_position_mm"] for g in R.segments])
            x = np.arange(rms.size, dtype=float)
            for label, color in gui.LABEL_COLORS.items():
                m = np.array([g["label"] == label for g in R.segments], bool)
                if m.any():
                    implot.plot_bars(label, gui._arr(x[m]), gui._arr(rms[m]), 0.8, implot.Spec(fill_color=gui.vec4(color)))
            implot.end_plot()


def run(R, start, horizon, every):
    app = Viewer(R, start, horizon, every)
    gui.run("puck model", app.gui)
