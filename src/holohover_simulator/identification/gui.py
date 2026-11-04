"""Shared Dear ImGui / ImPlot helpers (imgui-bundle) for the review and plot windows.

Run the windows on a machine with a display (not over ssh X forwarding: every frame is sent
as an image, a few frames per second at best).
"""
import contextlib

import numpy as np
from imgui_bundle import hello_imgui, imgui, immapp, implot

GLIDE = (0.17, 0.63, 0.17, 1.0)
WALL = (1.0, 0.5, 0.05, 1.0)
HIT = (0.89, 0.2, 0.2, 1.0)
MEAS = (0.78, 0.78, 0.78, 1.0)
MODEL = (0.95, 0.3, 0.3, 1.0)
PRED = (0.3, 0.6, 1.0, 1.0)
WALL_LINE = (0.4, 0.6, 1.0, 1.0)
FLIGHT = (0.55, 0.55, 0.55, 1.0)
CURSOR = (1.0, 0.25, 0.25, 1.0)
YAW = (0.7, 0.5, 0.9, 1.0)
LABEL_COLORS = {"glide": GLIDE, "wall_impact": WALL, "hit": HIT}
X1, Y1 = implot.ImAxis_.x1, implot.ImAxis_.y1


def vec4(c, alpha=None):
    return imgui.ImVec4(c[0], c[1], c[2], c[3] if alpha is None else alpha)


def line(label, x, y, color, weight=1.0, legend=True):
    flags = 0 if legend else implot.ItemFlags_.no_legend  # NaN breaks the line (nan_joined)
    implot.plot_line(label, _arr(x), _arr(y), implot.Spec(line_color=vec4(color), line_weight=weight, flags=flags))


def dots(label, x, y, color, size=1.5, legend=True):
    implot.plot_scatter(label, _arr(x), _arr(y),
                        implot.Spec(marker=implot.Marker_.circle, marker_size=size, marker_line_color=vec4(color),
                                    marker_fill_color=vec4(color), flags=0 if legend else implot.ItemFlags_.no_legend))


def scatter(label, x, y, size=3.0):
    """Markers in the next colormap color."""
    implot.plot_scatter(label, _arr(x), _arr(y), implot.Spec(marker=implot.Marker_.circle, marker_size=size))


def vlines(label, xs, color, weight=1.0, legend=True):
    if len(xs):
        implot.plot_inf_lines(label, _arr(xs), implot.Spec(line_color=vec4(color), line_weight=weight,
                                                           flags=0 if legend else implot.ItemFlags_.no_legend))


def hline(label, y, color, weight=1.0):
    implot.plot_inf_lines(label, _arr([y]), implot.Spec(line_color=vec4(color), line_weight=weight,
                                                        flags=implot.InfLinesFlags_.horizontal))


def spans(intervals, alpha=0.18):
    """Shaded vertical bands [(t0, t1, color)] behind the next items of the current plot."""
    lim = implot.get_plot_limits()
    dl = implot.get_plot_draw_list()
    implot.push_plot_clip_rect()
    for t0, t1, color, *a in intervals:
        if t1 < lim.x.min or t0 > lim.x.max:
            continue
        p0 = implot.plot_to_pixels(t0, lim.y.max)
        p1 = implot.plot_to_pixels(t1, lim.y.min)
        dl.add_rect_filled(p0, p1, imgui.get_color_u32(vec4(color, a[0] if a else alpha)))
    implot.pop_plot_clip_rect()


def nan_joined(pieces):
    """One (x, y) pair of arrays from [(x, y)] pieces, separated by NaN (drawn as separate lines)."""
    if not pieces:
        return np.empty(0), np.empty(0)
    xs, ys = [], []
    for x, y in pieces:
        xs += [np.asarray(x, float), [np.nan]]
        ys += [np.asarray(y, float), [np.nan]]
    return np.concatenate(xs), np.concatenate(ys)


def _arr(x):
    a = np.ascontiguousarray(x, dtype=float).ravel()
    return a if a.strides == (a.itemsize,) else a.copy()  # size-1 slices keep their old stride


def key(k, ctrl=False):
    """Key pressed this frame (not while typing in a text field)."""
    if imgui.get_io().want_text_input:
        return False
    return imgui.is_key_pressed(k, repeat=False) and imgui.get_io().key_ctrl == ctrl


class TimeAxis:
    """Shared, linked time axis of several plots, with a time cursor that the view follows."""

    def __init__(self, t0, t1, width=None):
        self.t_min, self.t_max = float(t0), float(t1)
        self.lo = implot.BoxedValue(self.t_min)
        self.hi = implot.BoxedValue(self.t_min + min(width or np.inf, self.t_max - self.t_min))
        self.cursor = self.t_min
        self._press = None

    def setup(self):
        """Call after begin_plot, before the items."""
        implot.setup_axis_links(X1, self.lo, self.hi)

    def show(self, t0, t1):
        self.lo.value, self.hi.value = float(t0), float(t1)

    def show_all(self):
        self.show(self.t_min, self.t_max)

    def follow(self, t, edge=0.05):
        """Pan (same zoom) so that t stays inside the view."""
        lo, hi = self.lo.value, self.hi.value
        w = hi - lo
        m = edge * w
        if lo + m <= t <= hi - m or w >= self.t_max - self.t_min:
            return
        shift = t - (hi - m) if t > hi - m else t - (lo + m)
        shift = float(np.clip(shift, self.t_min - lo, self.t_max - hi))
        self.show(lo + shift, hi + shift)

    def clicked(self):
        """Time of a click (press and release without dragging) in the current plot, or None."""
        if not implot.is_plot_hovered():
            return None
        if imgui.is_mouse_clicked(0):
            self._press = imgui.get_mouse_pos()
        if imgui.is_mouse_released(0) and self._press is not None:
            p, q = self._press, imgui.get_mouse_pos()
            self._press = None
            if (p.x - q.x) ** 2 + (p.y - q.y) ** 2 < 16:
                return float(implot.get_plot_mouse_pos().x)
        return None

    def slider(self, label="##time"):
        """Full width slider for the cursor; returns True when moved."""
        imgui.set_next_item_width(-1)
        changed, t = imgui.slider_float(label, self.cursor, self.t_min, self.t_max, "t = %.3f s")
        if changed:
            self.cursor = float(t)
            self.follow(self.cursor)
        return changed


def plot_heights(n, reserve=0.0):
    """Height of each of n stacked plots filling the rest of the window."""
    avail = imgui.get_content_region_avail().y - reserve
    return max(80.0, avail / n - imgui.get_style().item_spacing.y)


def run(title, show_gui, confirm_exit=None, size=(1600, 950)):
    params = hello_imgui.RunnerParams()
    params.app_window_params.window_title = title
    params.app_window_params.window_geometry.size = size
    params.callbacks.show_gui = show_gui
    if confirm_exit is not None:
        params.callbacks.confirm_exit = confirm_exit
    params.ini_disable = True
    params.imgui_window_params.default_imgui_window_type = hello_imgui.DefaultImGuiWindowType.no_default_window
    params.fps_idling.fps_idle = 10.0
    immapp.run(params, immapp.AddOnsParams(with_implot=True))
    return params


@contextlib.contextmanager
def fullscreen(name):
    """The window that holds the whole app."""
    vp = imgui.get_main_viewport()
    imgui.set_next_window_pos(vp.work_pos)
    imgui.set_next_window_size(vp.work_size)
    flags = (imgui.WindowFlags_.no_decoration | imgui.WindowFlags_.no_move | imgui.WindowFlags_.no_saved_settings
             | imgui.WindowFlags_.no_bring_to_front_on_focus)
    imgui.begin(name, None, flags)
    yield  # no end() after an exception: it would hide the error with an imgui assert
    imgui.end()


def shall_exit():
    hello_imgui.get_runner_params().app_shall_exit = True


@contextlib.contextmanager
def headless(size=(1600, 950)):
    """ImGui + ImPlot contexts without a window (tests): yields frame(fn) that draws one frame."""
    imgui.create_context()
    implot.create_context()
    io = imgui.get_io()
    io.set_ini_filename(None)  # no imgui.ini
    io.display_size = imgui.ImVec2(*size)
    io.delta_time = 1.0 / 60.0
    io.backend_flags |= imgui.BackendFlags_.renderer_has_textures  # no renderer: textures stay in memory

    def frame(fn):
        imgui.new_frame()
        fn()
        imgui.render()

    try:
        yield frame
    finally:
        implot.destroy_context()
        imgui.destroy_context()
