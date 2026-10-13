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
    assert {g["label"] for g in segments} == {"glide", "wall_impact"}


def test_fit_recovers_the_parameters(synthetic_fit):
    out = synthetic_fit[4]
    got = {k: v["value"] for k, v in out["parameters"].items()}
    tol = {"c_v": 0.003, "g_x": 0.001, "g_y": 0.001, "c_w": 0.02, "c_wc": 0.03,
           "e_n": 0.005, "mu": 0.005, "e_t": 0.02}
    for k, true in synthetic.TRUE.items():
        assert got[k] == pytest.approx(true, abs=tol[k]), k
    assert out["fit"]["rms_position_mm"] < 0.6  # noise is 0.3 mm per axis
    assert out["fit"]["data"] == synthetic_fit[0].kind


def test_review_and_plot_run_headless(synthetic_fit, tmp_path):
    import matplotlib
    matplotlib.use("Agg")
    import plot
    import review

    rec, walls, events, segments, out, seg_yaml, rec_seg = synthetic_fit
    seg_file = tmp_path / "segments.yaml"
    data.save_yaml(str(seg_file), segment.to_yaml(rec_seg, walls, events, segments))
    # review: select, relabel, delete, add, undo and save through the handlers
    rv = review.Review(rec_seg, str(seg_file), data.load_config())
    n = len(rv.segments)
    g = dict(rv.segments[0])  # copy: the review edits its segments in place

    class Ev:
        def __init__(self, **kw):
            self.__dict__.update(kw)

    rv.on_click(Ev(inaxes=rv.ax_s, button=1, xdata=(g["t0"] + g["t1"]) / 2))
    assert rv.selected == 0
    # the click moved the time slider: red dot at the puck position at that time
    i = np.searchsorted(rec_seg.t, rv.slider.val)
    np.testing.assert_allclose(np.ravel(rv.cursor_dot.get_data()), rec_seg.p[i])
    rv.on_key(Ev(key="w"))
    assert rv.segments[0]["label"] == "wall_impact"
    rv.on_key(Ev(key="d"))
    assert len(rv.segments) == n - 1
    rv.on_key(Ev(key="u"))
    rv.on_key(Ev(key="u"))
    assert len(rv.segments) == n and rv.segments[0]["label"] == g["label"]
    rv.on_key(Ev(key="d"))
    rv.on_span(g["t0"], g["t1"])  # re-add it as a new segment
    rv.on_key(Ev(key="s"))
    saved = data.load_yaml(str(seg_file))["segments"]
    assert len(saved) == n and [s["id"] for s in saved] == list(range(n))
    # plots
    plot.run(rec, seg_yaml, out, data.load_config(), save_dir=str(tmp_path / "plots"), segment_id=0)
    assert (tmp_path / "plots" / "summary.png").exists()
    hs, rms, _ = plot.prediction_error(plot.Results(rec, seg_yaml, out, data.load_config()), 0.5)
    assert np.nanmax(rms) < 10.0  # mm over 0.5 s on noise-only data (1 cm/s EKF velocity noise: ~5 mm)
