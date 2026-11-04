"""End to end: synthetic bag with known parameters -> segment -> fit recovers them."""
import numpy as np
import pytest

import data
import fit
import segment
import synthetic


@pytest.fixture(scope="module")
def bag(tmp_path_factory):
    path = tmp_path_factory.mktemp("synth") / "synth_bag"
    synthetic.write(str(path), data.load_config(), duration=40.0, seed=3)
    return str(path)


@pytest.fixture(scope="module", params=["state", "pose"])
def synthetic_fit(request, bag, tmp_path_factory):
    """segment (on the raw pose) and fit (on the source under test)."""
    cfg = data.with_source(data.load_config(), request.param)
    rec_seg = data.load_for_segmentation(bag, cfg)
    walls, events, segments = segment.auto_segment(rec_seg, cfg)
    seg_file = tmp_path_factory.mktemp("seg") / "segments.yaml"
    data.save_yaml(str(seg_file), segment.to_yaml(rec_seg, walls, events, segments))
    rec = data.load(bag, cfg)
    seg_yaml = data.load_segments(str(seg_file), rec)
    pr, res = fit.run(rec, seg_yaml, cfg, verbose=0)
    return rec, walls, events, segments, fit.report(pr, res), seg_yaml, rec_seg


def test_loader_uses_the_table_frame(synthetic_fit):
    rec, rec_seg = synthetic_fit[0], synthetic_fit[6]
    assert rec_seg.kind == "pose"  # segmentation always on the raw pose when the bag has it
    x0, x1, y0, y1 = synthetic.BOX
    assert rec.p[:, 0].min() > x0 - 0.01 and rec.p[:, 0].max() < x1 + 0.01
    assert rec.p[:, 1].min() > y0 - 0.01 and rec.p[:, 1].max() < y1 + 0.01
    assert 0.9 < rec.yaw_ok.mean() < 1.0  # the flipped orientations are rejected
    assert len(rec.chunks) == 2  # the gap
    if rec.kind == "state":  # EKF velocities, rotated into the table frame
        assert np.nanmax(np.abs(rec.v[:, 0])) < 2.0 and rec.measured_velocity


def test_segmentation_finds_the_walls_and_impacts(synthetic_fit):
    _, walls, events, segments, *_ = synthetic_fit
    offsets = {w["name"]: w["offset"] for w in walls}
    truth = {w["name"]: w["offset"] for w in synthetic.walls()}
    for name, off in offsets.items():
        assert off == pytest.approx(truth[name], abs=0.005)
    assert sum(e["kind"] == "wall" for e in events) >= 20
    assert {g["label"] for g in segments} == {"glide", "wall_impact", "hit"}  # the random shots are hits


def test_fit_recovers_the_parameters(synthetic_fit):
    out = synthetic_fit[4]
    got = {k: v["value"] for k, v in out["parameters"].items()}
    tol = {"c_v": 0.003, "g_x": 0.001, "g_y": 0.001, "c_w": 0.02, "c_wc": 0.03,
           "e_n": 0.005, "mu": 0.005, "e_t": 0.02}
    for k, true in synthetic.TRUE.items():
        assert got[k] == pytest.approx(true, abs=tol[k]), k
    assert out["fit"]["rms_position_mm"] < 0.6  # noise is 0.3 mm per axis
    assert out["fit"]["data"] == synthetic_fit[0].kind


def test_review_state(synthetic_fit, tmp_path):
    import review

    rec, walls, events, segments, out, seg_yaml, rec_seg = synthetic_fit
    seg_file = tmp_path / "segments.yaml"
    data.save_yaml(str(seg_file), segment.to_yaml(rec_seg, walls, events, segments))
    s = review.ReviewState(rec_seg, str(seg_file))
    n = len(s.segments)
    g0, g1 = dict(s.segments[0]), dict(s.segments[1])
    # the cursor selects the segment under it, nothing outside the segments
    s.select_at((g1["t0"] + g1["t1"]) / 2)
    assert s.selected == 1
    s.select_at((g0["t0"] + g0["t1"]) / 2)
    assert s.selected == 0
    s.select_at(rec_seg.t[0] - 1.0)
    assert s.selected is None
    # where two segments touch, the selected one stays selected
    s.select(1)
    s.select_at(g1["t0"])
    assert s.selected == 1 or g1["t0"] != g0["t1"]
    # relabel, delete, undo
    s.select(0)
    s.set_label("hit")
    assert s.segments[0]["label"] == "hit" and s.dirty
    s.set_label("wall_impact")
    s.delete()
    assert len(s.segments) == n - 1 and s.selected is None
    for _ in range(3):
        s.undo()
    assert len(s.segments) == n and s.segments[0]["label"] == g0["label"]
    # edges stay between the neighbours
    s.select(1)
    s.begin_edit()
    s.set_bounds(g0["t0"] - 5.0, g1["t1"])
    assert s.segments[1]["t0"] == pytest.approx(s.segments[0]["t1"])
    s.undo()
    assert s.segments[1]["t0"] == g1["t0"]
    # new segments do not overlap
    s.select(0)
    s.delete()
    assert not s.add(g0["t0"], g1["t1"])  # overlaps segment 1
    assert s.add_at(g0["t0"]) and s.segments[0]["t0"] == pytest.approx(g0["t0"])
    assert s.segments[0]["t1"] <= s.segments[1]["t0"]
    assert not s.add_at(s.segments[0]["t0"] + 0.01)  # inside a segment
    s.save()
    saved = data.load_yaml(str(seg_file))["segments"]
    assert len(saved) == n and [g["id"] for g in saved] == list(range(n)) and not s.dirty


def test_windows_draw_headless(synthetic_fit, tmp_path):
    """Both windows draw frames without a display; the review handles its keys and the cursor."""
    import gui
    import plot
    import review
    import viewer

    rec, walls, events, segments, out, seg_yaml, rec_seg = synthetic_fit
    seg_file = tmp_path / "segments.yaml"
    data.save_yaml(str(seg_file), segment.to_yaml(rec_seg, walls, events, segments))
    R = plot.Results(rec, seg_yaml, out, data.load_config())
    with gui.headless() as frame:
        app = review.ReviewApp(rec_seg, str(seg_file))
        frame(app.gui)
        app.goto(1)
        frame(app.gui)
        g = app.s.segments[0]
        assert app.s.selected == 0 and app.axis.lo.value < g["t0"] < g["t1"] < app.axis.hi.value
        # zoomed in, the view follows the cursor with the same zoom, and stops at the end
        app.axis.show(rec_seg.t[0], rec_seg.t[0] + 2.0)
        app.move_cursor(rec_seg.t[0] + 10.0)
        assert app.axis.hi.value - app.axis.lo.value == pytest.approx(2.0)
        assert app.axis.lo.value < app.axis.cursor < app.axis.hi.value
        app.move_cursor(rec_seg.t[-1] + 1.0)
        assert app.axis.hi.value == pytest.approx(rec_seg.t[-1])
        app.s.select(0)
        app.s.set_label("hit")
        app.request_quit()  # unsaved changes: asks first
        frame(app.gui)
        assert not app.quit_now and not app.confirm_exit()
        app.s.save()
        assert app.confirm_exit()

        v = viewer.Viewer(R, None, 0.5, 0.05)
        v.fan = True
        frame(v.gui)
        for tab in (v.timeline_tab, v.summary_tab):
            def draw(tab=tab):
                with gui.fullscreen("viewer"):
                    tab()
            frame(draw)
        v._set_segment(len(R.segments))  # wraps around
        assert v.i == 0


def test_plots_saved(synthetic_fit, tmp_path):
    import plot

    rec, *_, out, seg_yaml, _ = synthetic_fit
    plot.run(rec, seg_yaml, out, data.load_config(), save_dir=str(tmp_path / "plots"), segment_id=0)
    assert (tmp_path / "plots" / "summary.png").exists()
    hs, rms, _ = plot.prediction_error(plot.Results(rec, seg_yaml, out, data.load_config()), 0.5)
    assert np.nanmax(rms) < 10.0  # mm over 0.5 s on noise-only data (1 cm/s EKF velocity noise: ~5 mm)
