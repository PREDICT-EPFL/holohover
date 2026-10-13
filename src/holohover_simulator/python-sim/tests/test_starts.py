"""The stored evaluation starts (config evaluation.starts): evaluate's fixed test set."""

import json
from collections import Counter

import numpy as np
from mcap.reader import make_reader

from conftest import SIM_DIR
from simulation.setup import generate_starts, load_starts, start_state, write_starts
from viz.view_starts import write


def test_file_is_reproducible(config):
    """The file is what its seed and sampling parameters draw, and the config still samples that
    way: redrawing it (main.py starts --force) would not change the starts."""
    with open(SIM_DIR / config["evaluation"]["starts"]) as f:
        stored = json.load(f)
    assert stored["sampling"] == config["evaluation"]["sampling"]
    assert generate_starts(config, len(stored["starts"]), stored["seed"]) == stored


def test_round_trip_is_exact(config, tmp_path):
    contents = generate_starts(config, 20, seed=3)
    write_starts(tmp_path / "starts.json", contents)
    assert load_starts(tmp_path / "starts.json") == contents["starts"]


def test_start_state(config):
    """Robot 1 and the puck from the start, robot 2 from the config."""
    start = load_starts(SIM_DIR / config["evaluation"]["starts"])[0]
    state = start_state(config, start)
    assert state.shape == (18,)
    np.testing.assert_array_equal(state[:6], start["robot1"])
    np.testing.assert_array_equal(state[12:], start["puck"])
    np.testing.assert_array_equal(state[6:8], config["robot"]["robot2"]["park"])


def test_view_starts_mcap(config, tmp_path):
    """viz/view_starts.py: the static scene once, then one frame per start, a second apart."""
    starts = load_starts(SIM_DIR / config["evaluation"]["starts"])[5:8]
    path = write(config, starts, first=5, path=tmp_path / "starts.mcap")
    with open(path, "rb") as f:
        messages = [(channel.topic, message) for _, channel, message in make_reader(f).iter_messages()]
    counts = Counter(topic for topic, _ in messages)
    assert counts["/scene/table"] == counts["/scene/bodies"] == 1
    assert all(counts[topic] == 3 for topic in ("/tf", "/puck/prediction", "/events", "/puck/state"))
    puck = [json.loads(m.data) for topic, m in messages if topic == "/puck/state"]
    assert [p["t"] for p in puck] == [5.0, 6.0, 7.0]
    assert [[p[k] for k in ("x", "y", "theta", "vx", "vy", "omega")] for p in puck] == [s["puck"] for s in starts]
    times = [m.log_time for topic, m in messages if topic == "/tf"]
    assert np.diff(times).tolist() == [1_000_000_000] * 2
