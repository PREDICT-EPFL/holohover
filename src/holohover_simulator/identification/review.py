"""Window to check and correct the segments (Dear ImGui / ImPlot).

Left: the segment list, the label, the edit buttons and the warnings of the selected segment.
Middle: speed, position and yaw over time with the segments (green: glide, orange: wall impact,
red: hit, not fitted), gaps in the data (dark), the detected events (blue: wall impact, red:
hit, grey: flight, an acceleration spike without velocity change) and the time cursor (red).
Right: the table with the walls, the selected segment and the puck at the cursor time.

    mouse wheel / right drag   zoom (time plots stay aligned), left drag: pan, double click: fit
    click on a time plot       move the cursor there (selects the segment there)
    drag the red line          move the cursor;  slider at the bottom: the same
    drag a selected edge       change the start / end of the selected segment
    g / w / h                  label glide / wall_impact / hit (also relabels the selected segment)
    i                          new segment at the cursor (0.5 s, stops at the next segment)
    d, delete                  delete the selected segment
    n / b                      next / previous segment (zooms to it)
    a                          show the whole recording
    left / right               move the cursor by 0.02 s (shift: 0.2 s)
    u, ctrl+z                  undo
    s, ctrl+s                  save segments.yaml
    q                          quit (asks when there are unsaved changes)
"""
import copy
import os

import numpy as np
from imgui_bundle import imgui, implot

import data
import gui
import model

LABELS = ("glide", "wall_impact", "hit")
LABEL_KEYS = {imgui.Key.g: "glide", imgui.Key.w: "wall_impact", imgui.Key.h: "hit"}
MIN_LENGTH = 0.02  # [s]
NEW_LENGTH = 0.5   # [s] length of a new segment


class ReviewState:
    """The segments being reviewed, the selection and the undo history (no GUI)."""

    def __init__(self, rec, seg_file):
        self.rec, self.seg_file = rec, seg_file
        self.doc = data.load_segments(seg_file, rec)
        self.walls = model.walls_from_yaml(self.doc.get("walls", []))
        self.events = self.doc.get("events", [])
        self.segments = sorted((dict(g) for g in self.doc.get("segments", [])), key=lambda g: g["t0"])
        self.label = "glide"
        self.selected = None
        self.history = []
        self.dirty = False
        self.message = ""

    # queries
    def at(self, t):
        """Index of the segment at time t (the selected one where two segments touch), or None."""
        s = self.selected
        if s is not None and self.segments[s]["t0"] <= t <= self.segments[s]["t1"]:
            return s
        return next((i for i, g in enumerate(self.segments) if g["t0"] <= t <= g["t1"]), None)

    def counts(self):
        return {k: sum(g["label"] == k for g in self.segments) for k in LABELS}

    def check(self, g):
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
        r = self.rec
        for a, b in zip(r.chunks[:-1], r.chunks[1:]):
            if g["t0"] < r.t[b[0]] and r.t[a[1] - 1] < g["t1"]:
                msgs.append("spans a gap in the data")
                break
        return msgs

    # selection
    def select(self, i):
        self.selected = i

    def select_at(self, t):
        self.selected = self.at(t)

    def step(self, k):
        """Select the next (k = 1) or previous (k = -1) segment."""
        if not self.segments:
            return None
        if self.selected is None:
            self.selected = 0 if k > 0 else len(self.segments) - 1
        else:
            self.selected = (self.selected + k) % len(self.segments)
        self.message = f"segment {self.selected + 1}/{len(self.segments)}"
        return self.segments[self.selected]

    # edits
    def _push(self):
        self.history.append((copy.deepcopy(self.segments), self.selected))
        self.dirty = True

    def set_label(self, label):
        self.label = label
        g = self.segments[self.selected] if self.selected is not None else None
        if g is not None and g["label"] != label:
            self._push()
            g["label"] = label
            self.message = f"relabelled to {label}"

    def delete(self):
        if self.selected is None:
            return
        self._push()
        del self.segments[self.selected]
        self.selected = None
        self.message = "deleted"

    def undo(self):
        if not self.history:
            self.message = "nothing to undo"
            return
        self.segments, self.selected = self.history.pop()
        self.dirty = True
        self.message = "undone"

    def add(self, t0, t1):
        """New segment [t0, t1] with the current label; False when it overlaps one."""
        t0, t1 = max(float(t0), float(self.rec.t[0])), min(float(t1), float(self.rec.t[-1]))
        if t1 - t0 < MIN_LENGTH:
            self.message = "too short"
            return False
        if any(t0 < g["t1"] and g["t0"] < t1 for g in self.segments):
            self.message = "overlaps an existing segment"
            return False
        self._push()
        g = {"id": None, "label": self.label, "t0": round(t0, 4), "t1": round(t1, 4)}
        self.segments.append(g)
        self.segments.sort(key=lambda s: s["t0"])
        self.selected = self.segments.index(g)
        self.message = f"new {self.label}"
        return True

    def add_at(self, t):
        """New segment from t, NEW_LENGTH long or up to the next segment."""
        if self.at(t) is not None:
            self.message = "the cursor is inside a segment"
            return False
        nxt = min((g["t0"] for g in self.segments if g["t0"] > t), default=np.inf)
        return self.add(t, min(t + NEW_LENGTH, nxt))

    def begin_edit(self):
        """Call once before a series of set_bounds (one undo step)."""
        self._push()

    def set_bounds(self, t0, t1):
        """New start and end of the selected segment, kept between its neighbours."""
        i = self.selected
        if i is None:
            return
        lo = self.segments[i - 1]["t1"] if i > 0 else float(self.rec.t[0])
        hi = self.segments[i + 1]["t0"] if i + 1 < len(self.segments) else float(self.rec.t[-1])
        g = self.segments[i]
        t0 = float(np.clip(t0, lo, hi - MIN_LENGTH))
        t1 = float(np.clip(t1, t0 + MIN_LENGTH, hi))
        g["t0"], g["t1"] = round(t0, 4), round(t1, 4)
        self.dirty = True

    def save(self):
        for i, g in enumerate(self.segments):
            g["id"] = i
        self.doc["segments"] = self.segments
        data.save_yaml(self.seg_file, self.doc,
                       header="Puck identification segments, times in s from t_ref (first puck stamp).\n"
                              "Edit with `identify.py review`. Labels: glide, wall_impact, hit (not fitted).")
        self.dirty = False
        self.message = f"saved {len(self.segments)} segments"
        print(f"saved {len(self.segments)} segments to {self.seg_file}")


class ReviewApp:
    SIDEBAR = 360

    def __init__(self, rec, seg_file):
        self.s = ReviewState(rec, seg_file)
        self.rec = r = rec
        self.axis = gui.TimeAxis(r.t[0], r.t[-1])
        self.editing = False   # dragging a segment edge (one undo step per drag)
        self.ask_quit = False
        self.quit_now = False
        self.scroll_list = False
        # static data, contiguous for ImPlot
        ok = r.yaw_ok
        self.yaw_ok = (r.t[ok], r.yaw[ok])
        self.yaw_bad = (r.t[~ok], r.yaw[~ok])
        self.gaps = [(r.t[a[1] - 1], r.t[b[0]], (0.0, 0.0, 0.0, 1.0), 0.45)
                     for a, b in zip(r.chunks[:-1], r.chunks[1:])]
        ev = self.s.events
        self.ev = {k: [e["t"] for e in ev if e["kind"] == k] for k in ("wall", "hit", "flight")}
        self.contacts = np.array([e["contact"] for e in ev if e["kind"] == "wall" and "contact" in e]).reshape(-1, 2)
        self.speed_max = float(min(3.0, np.nanpercentile(r.speed, 99.5) * 1.3))
        self.walls = []
        lo, hi = r.p.min(0) - 0.05, r.p.max(0) + 0.05
        for w in self.s.walls:
            n, d = np.asarray(w["normal"]), w["offset"]
            t = np.array([-n[1], n[0]])
            c = np.array([lo, hi, [lo[0], hi[1]], [hi[0], lo[1]]]) @ t
            self.walls.append((w["name"], np.outer([c.min(), c.max()], t) + d * n))

    # ------------------------------------------------------------------ actions
    def move_cursor(self, t):
        self.axis.cursor = float(np.clip(t, self.axis.t_min, self.axis.t_max))
        self.axis.follow(self.axis.cursor)
        self.s.select_at(self.axis.cursor)

    def goto(self, k):
        g = self.s.step(k)
        if g is None:
            return
        pad = max(0.5, 0.5 * (g["t1"] - g["t0"]))
        self.axis.show(g["t0"] - pad, g["t1"] + pad)
        self.axis.cursor = g["t0"]  # inside the selected segment: the selection stays
        self.scroll_list = True

    def request_quit(self):
        if self.s.dirty:
            self.ask_quit = True
        else:
            self.quit_now = True

    def confirm_exit(self):
        """Window close button: ask first when there are unsaved changes."""
        if not self.s.dirty:
            return True
        self.ask_quit = True
        return False

    def keys(self):
        for k, label in LABEL_KEYS.items():
            if gui.key(k):
                self.s.set_label(label)
        if gui.key(imgui.Key.d) or gui.key(imgui.Key.delete):
            self.s.delete()
        if gui.key(imgui.Key.i):
            self.s.add_at(self.axis.cursor)
        if gui.key(imgui.Key.n):
            self.goto(1)
        if gui.key(imgui.Key.b):
            self.goto(-1)
        if gui.key(imgui.Key.a):
            self.axis.show_all()
        if gui.key(imgui.Key.u) or gui.key(imgui.Key.z, ctrl=True):
            self.s.undo()
        if gui.key(imgui.Key.s) or gui.key(imgui.Key.s, ctrl=True):
            self.s.save()
        if gui.key(imgui.Key.q):
            self.request_quit()
        io = imgui.get_io()
        if not io.want_text_input:
            step = 0.2 if io.key_shift else 0.02
            if imgui.is_key_pressed(imgui.Key.right_arrow):
                self.move_cursor(self.axis.cursor + step)
            if imgui.is_key_pressed(imgui.Key.left_arrow):
                self.move_cursor(self.axis.cursor - step)

    # ------------------------------------------------------------------ drawing
    def gui(self):
        if self.quit_now:
            gui.shall_exit()
        with gui.fullscreen("review"):
            if not imgui.is_any_item_active():
                self.keys()
            if not imgui.is_mouse_down(0):
                self.editing = False
            imgui.begin_child("sidebar", imgui.ImVec2(self.SIDEBAR, 0))
            self.sidebar()
            imgui.end_child()
            imgui.same_line()
            imgui.begin_child("main", imgui.ImVec2(0, 0))
            self.main()
            imgui.end_child()
            self.quit_dialog()

    def sidebar(self):
        s = self.s
        imgui.text(os.path.basename(os.path.dirname(s.seg_file)) or s.seg_file)
        n = s.counts()
        total = sum(g["t1"] - g["t0"] for g in s.segments)
        imgui.text(f"{n['glide']} glides, {n['wall_impact']} wall impacts, {n['hit']} hits, {total:.1f} s")
        if s.dirty:
            imgui.text_colored(gui.vec4(gui.WALL), "unsaved changes")
        imgui.separator()
        imgui.text("label (new and selected segment)")
        for label, k in zip(LABELS, "gwh"):
            imgui.push_style_color(imgui.Col_.text, gui.vec4(gui.LABEL_COLORS[label]))
            if imgui.radio_button(f"{label} ({k})", s.label == label):
                s.set_label(label)
            imgui.pop_style_color()
            imgui.same_line()
        imgui.new_line()
        w = (self.SIDEBAR - 3 * imgui.get_style().item_spacing.x) / 3
        bw = imgui.ImVec2(w, 0)
        if imgui.button("new at cursor (i)", bw):
            s.add_at(self.axis.cursor)
        imgui.same_line()
        if imgui.button("delete (d)", bw):
            s.delete()
        imgui.same_line()
        if imgui.button("undo (u)", bw):
            s.undo()
        if imgui.button("< prev (b)", bw):
            self.goto(-1)
        imgui.same_line()
        if imgui.button("next (n) >", bw):
            self.goto(1)
        imgui.same_line()
        if imgui.button("show all (a)", bw):
            self.axis.show_all()
        if imgui.button("save (s)", imgui.ImVec2(-1, 0)):
            s.save()
        imgui.separator()
        if s.selected is not None:
            g = s.segments[s.selected]
            imgui.text_colored(gui.vec4(gui.LABEL_COLORS.get(g["label"], gui.MEAS)),
                               f"selected: {g['label']}  {g['t0']:.3f} - {g['t1']:.3f} s  ({g['t1'] - g['t0']:.2f} s)")
            for m in s.check(g):
                imgui.text_colored(gui.vec4(gui.WALL), f"warning: {m}")
        else:
            imgui.text_disabled("no segment selected")
        imgui.text_disabled(s.message or " ")
        imgui.separator()
        self.segment_table()
        if imgui.collapsing_header("help"):
            imgui.text_wrapped(__doc__.split("\n\n", 2)[2])

    def segment_table(self):
        s = self.s
        flags = (imgui.TableFlags_.scroll_y | imgui.TableFlags_.row_bg | imgui.TableFlags_.borders_inner_v
                 | imgui.TableFlags_.sizing_stretch_prop)
        height = imgui.get_content_region_avail().y - imgui.get_frame_height_with_spacing() - 4
        if not imgui.begin_table("segments", 4, flags, imgui.ImVec2(0, max(100.0, height))):
            return
        imgui.table_setup_scroll_freeze(0, 1)
        for name in ("#", "label", "start [s]", "length [s]"):
            imgui.table_setup_column(name)
        imgui.table_headers_row()
        for i, g in enumerate(s.segments):
            imgui.table_next_row()
            imgui.table_next_column()
            sel = i == s.selected
            if imgui.selectable(f"{i}##row", sel, imgui.SelectableFlags_.span_all_columns)[0]:
                s.select(i)
                pad = max(0.5, 0.5 * (g["t1"] - g["t0"]))
                self.axis.show(g["t0"] - pad, g["t1"] + pad)
                self.axis.cursor = g["t0"]
            if sel and self.scroll_list:
                imgui.set_scroll_here_y()
                self.scroll_list = False
            imgui.table_next_column()
            warn = " !" if s.check(g) else ""
            imgui.text_colored(gui.vec4(gui.LABEL_COLORS.get(g["label"], gui.MEAS)), g["label"] + warn)
            imgui.table_next_column()
            imgui.text(f"{g['t0']:.2f}")
            imgui.table_next_column()
            imgui.text(f"{g['t1'] - g['t0']:.2f}")
        imgui.end_table()

    def main(self):
        slider_h = imgui.get_frame_height_with_spacing()
        avail = imgui.get_content_region_avail()
        w_time = avail.x * 0.68
        imgui.begin_child("time", imgui.ImVec2(w_time, avail.y - slider_h))
        h = gui.plot_heights(3)
        r = self.rec
        self.time_plot("speed", "speed [m/s]", h, (0.0, self.speed_max),
                       lambda: gui.line("speed", r.t, r.speed, gui.MEAS, legend=False))
        self.time_plot("position", "position [m]", h, (float(np.nanmin(r.p)) - 0.05, float(np.nanmax(r.p)) + 0.05),
                       lambda: (gui.line("x", r.t, r.p[:, 0], gui.PRED), gui.line("y", r.t, r.p[:, 1], gui.WALL)))
        self.time_plot("yaw", "yaw [rad]", h, (-3.3, 3.3),
                       lambda: (gui.dots("yaw", *self.yaw_ok, gui.YAW, 1.0),
                                gui.dots("invalid", *self.yaw_bad, gui.HIT, 1.5)), x_label="time [s]")
        imgui.end_child()
        imgui.same_line()
        imgui.begin_child("table", imgui.ImVec2(0, avail.y - slider_h))
        self.table_plot()
        imgui.end_child()
        if self.axis.slider():
            self.s.select_at(self.axis.cursor)

    def time_plot(self, name, ylabel, height, ylim, items, x_label=None):
        if not implot.begin_plot(f"##{name}", imgui.ImVec2(-1, height), implot.Flags_.no_menus):
            return
        self.axis.setup()
        implot.setup_axis(gui.X1, x_label, 0 if x_label else implot.AxisFlags_.no_tick_labels)
        implot.setup_axis(gui.Y1, ylabel)
        implot.setup_axis_limits(gui.Y1, *ylim)
        implot.setup_legend(implot.Location_.north_east)
        s = self.s
        bands = [(g["t0"], g["t1"], gui.LABEL_COLORS.get(g["label"], gui.MEAS), 0.35 if i == s.selected else 0.12)
                 for i, g in enumerate(s.segments)]
        gui.spans(bands + self.gaps)
        gui.vlines("wall impact", self.ev["wall"], gui.WALL_LINE, legend=name == "speed")
        gui.vlines("hit", self.ev["hit"], gui.HIT, 1.5, legend=name == "speed")
        gui.vlines("flight", self.ev["flight"], gui.FLIGHT, legend=name == "speed")
        items()
        tool = False
        # edges of the selected segment
        if s.selected is not None:
            g = s.segments[s.selected]
            col = gui.vec4(gui.LABEL_COLORS.get(g["label"], gui.MEAS))
            for k, key in enumerate(("t0", "t1")):
                changed, x, _, hovered, held = implot.drag_line_x(1 + k, g[key], col, 3.0,
                                                                   out_hovered=False, out_held=False)
                tool |= bool(hovered or held)
                if changed:
                    if not self.editing:
                        s.begin_edit()
                        self.editing = True
                    if key == "t0":
                        s.set_bounds(x, g["t1"])
                    else:
                        s.set_bounds(g["t0"], x)
        changed, x, _, hovered, held = implot.drag_line_x(0, self.axis.cursor, gui.vec4(gui.CURSOR), 1.5,
                                                          out_hovered=False, out_held=False)
        tool |= bool(hovered or held)
        if changed:
            self.move_cursor(x)
        t = self.axis.clicked()
        if t is not None and not tool:
            self.move_cursor(t)
        implot.end_plot()

    def table_plot(self):
        r = self.rec
        i = int(np.clip(np.searchsorted(r.t, self.axis.cursor), 0, r.t.size - 1))
        imgui.text(f"t = {r.t[i]:.2f} s   x = {r.p[i, 0]:.3f}, y = {r.p[i, 1]:.3f} m   speed {r.speed[i]:.2f} m/s")
        if not implot.begin_plot("##table", imgui.ImVec2(-1, -1), implot.Flags_.equal | implot.Flags_.no_menus):
            return
        implot.setup_axes("x [m]", "y [m]")
        lo, hi = np.nanmin(r.p, 0), np.nanmax(r.p, 0)
        implot.setup_axes_limits(lo[0] - 0.1, hi[0] + 0.1, lo[1] - 0.1, hi[1] + 0.1)
        implot.setup_legend(implot.Location_.north_east)
        gui.line("path", r.p[:, 0], r.p[:, 1], (0.45, 0.45, 0.45, 1.0), legend=False)
        for name, pts in self.walls:
            gui.line("walls", pts[:, 0], pts[:, 1], gui.WALL_LINE, 2.0)
            implot.plot_text(name, *pts.mean(0))
        if self.contacts.size:
            gui.dots("wall contacts", self.contacts[:, 0], self.contacts[:, 1], gui.WALL_LINE, 2.5)
        s = self.s
        if s.selected is not None:
            g = s.segments[s.selected]
            idx = r.window(g["t0"], g["t1"])
            gui.line("selected segment", r.p[idx, 0], r.p[idx, 1], gui.LABEL_COLORS.get(g["label"], gui.MEAS), 2.5)
        trail = r.window(r.t[i] - 0.5, r.t[i])
        gui.line("last 0.5 s", r.p[trail, 0], r.p[trail, 1], gui.CURSOR, 1.5, legend=False)
        gui.dots("puck at the cursor", [r.p[i, 0]], [r.p[i, 1]], gui.CURSOR, 7.0)
        implot.end_plot()

    def quit_dialog(self):
        if self.ask_quit:
            imgui.open_popup("unsaved changes")
            self.ask_quit = False
        if imgui.begin_popup_modal("unsaved changes", None, imgui.WindowFlags_.always_auto_resize)[0]:
            imgui.text("The segments have unsaved changes.")
            if imgui.button("save and quit"):
                self.s.save()
                self.quit_now = True
                imgui.close_current_popup()
            imgui.same_line()
            if imgui.button("quit without saving"):
                self.s.dirty = False
                self.quit_now = True
                imgui.close_current_popup()
            imgui.same_line()
            if imgui.button("cancel") or gui.key(imgui.Key.escape):
                imgui.close_current_popup()
            imgui.end_popup()


def run(rec, seg_file, cfg):
    app = ReviewApp(rec, seg_file)
    gui.run(f"segments - {seg_file}", app.gui, app.confirm_exit)
