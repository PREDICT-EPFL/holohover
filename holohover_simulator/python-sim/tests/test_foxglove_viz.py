"""Foxglove export: topics, timing, strict JSON and decodable protobuf in the written MCAP."""

import json

import numpy as np
import pytest
from mcap.reader import make_reader

from foxglove_viz import ComputeTimeline, write_mcap

N_STEPS, N_PLAN, N_FUTURE, N_SHOT = 21, 20, 60, 8
START_NS = 1_700_000_000_000_000_000
LABELS = ["direct", "L"]

LOG = {
    "outcome": "goal", "t_goal": 0.105, "contacts": [0.075],
    "plan_history": [[0.05, "direct", 0.3], [0.06, "direct", 0.29], [0.1, "L", 0.2]],
    "updates": [{"t": 0.05, "outer": True, "chosen": "direct", "t_total": 0.012, "results": {
        "direct": {"valid": True, "cost": 1.0, "score": 1.0, "tf": 0.3, "t_wall": 0.010, "iters": 12, "cold": False},
        "L": {"valid": False, "cost": float("inf"), "score": float("nan"), "tf": 0.4, "t_wall": 0.002,
              "iters": 50, "cold": True},
    }}],
}


def theta(k):
    return -2.0 + 0.2 * k  # crosses zero and both signs


def episode(config, shot=True):
    """A run_shot.py-style dict (or simulation.run_sim-style with shot=False) with known values."""
    n = N_STEPS
    states = np.zeros((n, 18))
    states[:, 0] = 1.8 - 0.01 * np.arange(n)
    states[:, 1] = 0.5
    states[:, 2] = [theta(k) for k in range(n)]
    states[:, 12:14] = [1.0, 0.5]
    modes = np.zeros(n, dtype=int)
    modes[5:] = 1
    scores = np.zeros((n, 2), dtype=int)
    scores[10:, 0] = 1
    data = {
        "states": states,
        "input_sequence": np.ones((n, 3 if shot else 6)),
        "path_plans": np.zeros((n, N_PLAN, 9)),
        "tfs": np.linspace(0.5, 0.3, n),
        "modes": modes,
        "scores": scores,
        "future_trajs": np.zeros((n, N_FUTURE, 18)),
        "config": config,
    }
    if shot:
        robot = np.full((n, len(LABELS), N_PLAN + 1, 2), np.nan)
        robot[:, 0] = np.linspace([1.8, 0.5], [1.2, 0.5], N_PLAN + 1)
        data["candidates"] = {
            "labels": LABELS,
            "robot": robot,
            "shot": np.full((n, len(LABELS), N_SHOT, 2), np.nan),
            "score": np.tile([1.0, np.nan], (n, 1)),
            "valid": np.tile([True, False], (n, 1)),
            "chosen": np.where(modes == 1, 0, -1),
        }
    return data


def read(path):
    with open(path, "rb") as f:
        reader = make_reader(f)
        messages = [(channel.topic, schema, message) for schema, channel, message in reader.iter_messages()]
        metadata = {m.name: m.metadata for m in reader.iter_metadata()}
    with open(path, "rb") as f:
        attachments = [a.name for a in make_reader(f).iter_attachments()]
    return messages, metadata, attachments


def protobuf_class(schema):
    """Message class from the FileDescriptorSet stored in the MCAP schema."""
    descriptor_pb2 = pytest.importorskip("google.protobuf.descriptor_pb2")
    from google.protobuf import descriptor_pool, message_factory

    pool = descriptor_pool.DescriptorPool()
    for file in descriptor_pb2.FileDescriptorSet.FromString(schema.data).file:
        pool.Add(file)
    return message_factory.GetMessageClass(pool.FindMessageTypeByName(schema.name))


def strict_json(data):
    def reject(constant):
        raise ValueError(f"non-standard JSON constant {constant}")

    return json.loads(data, parse_constant=reject)


@pytest.fixture(scope="module")
def shot_file(config, tmp_path_factory):
    path = tmp_path_factory.mktemp("mcap") / "shot.mcap"
    write_mcap(episode(config), path, log=LOG, metadata={"scenario": "test"}, start_ns=START_NS)
    return read(path)


def test_topics_and_counts(shot_file):
    messages, _, _ = shot_file
    count = {}
    for topic, _, _ in messages:
        count[topic] = count.get(topic, 0) + 1
    per_step = ["/tf", "/robot1/state", "/robot2/state", "/puck/state", "/robot1/control", "/status",
                "/robot1/plan", "/puck/prediction", "/robot2/prediction", "/puck/trail", "/shot_mpc/scores",
                "/shot_mpc/candidates/direct", "/shot_mpc/candidates/L"]
    assert {t: count.get(t) for t in per_step} == {t: N_STEPS for t in per_step}
    assert count["/scene/table"] == count["/scene/bodies"] == count["/shot_mpc/solves"] == 1
    # mode DEF->ATT, goal, contact, plans direct and L, outcome
    assert count["/events"] == 6
    compute = [f"/shot_mpc/compute/{row}" for row in ("selector", *LABELS)]
    assert {t: count.get(t) for t in compute} == {t: 2 for t in compute}  # busy, then idle
    assert set(count) == set(per_step) | set(compute) | {"/scene/table", "/scene/bodies", "/shot_mpc/solves",
                                                         "/events"}


def test_compute_timeline():
    """Bars as wide as the wall time: parallel sequences start with the update, serial ones in turn, and
    a bar still running at the next one on its row is cut there. Failed solves and updates that solved
    nothing are marked. Older logs: outer or inner, warm or cold."""
    def update(t, kind, solves, t_wall, skip=None):
        """solves: label -> (wall time, start, valid)."""
        results = {name: {"t_wall": w, "cold": start != "warm", "start": start, "valid": valid}
                   for name, (w, start, valid) in solves.items()}
        return {"t": t, "outer": kind == "outer", "kind": kind, "parallel": len(solves) > 1, "skip": skip,
                "t_total": sum(w for w, _, _ in solves.values()), "t_wall": t_wall, "results": results}

    def rows(items):
        return [(round(t, 6), row, m.state, None if m.ms is None else round(m.ms, 6)) for t, row, m in items]

    timeline = ComputeTimeline()
    items = timeline.add(update(0.0, "outer", {"direct": (0.02, "warm", True), "L": (0.08, "cold", False)}, 0.085))
    items += timeline.add(update(0.05, "inner", {"direct": (0.004, "warm+cold", True)}, 0.005))
    items += timeline.flush(0.06)
    assert timeline.flush(0.08) == []  # L ends at 0.08: an update could still start then
    items += timeline.add(update(0.2, "outer", {}, 0.0001, skip="reach"))
    items += timeline.flush()
    assert rows(items) == [
        (0.0, "selector", "outer", 85.0), (0.0, "direct", "warm", 20.0), (0.0, "L", "cold fail", 80.0),
        (0.05, "selector", "inner", 5.0),  # the outer update was still running: no idle
        (0.02, "direct", "idle", None), (0.05, "direct", "warm+cold", 4.0),
        (0.054, "direct", "idle", None), (0.055, "selector", "idle", None),
        (0.2, "selector", "no shot (reach)", 0.1),
        (0.08, "L", "idle", None), (0.2001, "selector", "idle", None),
    ]

    old = {"t": 0.1, "outer": True, "t_total": 0.03,
           "results": {"direct": {"t_wall": 0.01, "cold": True, "valid": False},
                       "L": {"t_wall": 0.02, "cold": False, "valid": True}}}
    assert rows(ComputeTimeline().add(old)) == [
        (0.1, "selector", "outer", 30.0), (0.1, "direct", "cold fail", 10.0), (0.11, "L", "warm", 20.0)]
    assert rows(ComputeTimeline().add({"t": 0.3, "outer": True, "t_total": 0.0, "results": {}})) == [
        (0.3, "selector", "no shot", 0.0)]


def test_time_order_and_step(shot_file, config):
    messages, _, _ = shot_file
    times = [message.log_time for _, _, message in messages]
    assert times == sorted(times) and times[0] == START_NS
    tf_times = np.array([message.log_time for topic, _, message in messages if topic == "/tf"])
    assert np.allclose(np.diff(tf_times), 1e9 / config["simulation"]["hz"], atol=1)


def test_json_is_strict_and_non_finite_is_null(shot_file):
    messages, _, _ = shot_file
    decoded = {}
    for topic, schema, message in messages:
        if schema.encoding == "jsonschema":
            decoded.setdefault(topic, []).append(strict_json(message.data))
    solve = decoded["/shot_mpc/solves"][0]
    assert solve["results"]["L"]["cost"] is None and solve["results"]["L"]["score"] is None
    assert solve["t_total_ms"] == pytest.approx(12.0)
    assert decoded["/shot_mpc/scores"][-1] == {"t": pytest.approx(0.2), "chosen": "direct",
                                               "scores": {"direct": 1.0, "L": None},
                                               "valid": {"direct": True, "L": False}}
    status = decoded["/status"]
    assert status[0]["mode_name"] == "DEF" and status[0]["time_to_impact"] is None
    assert status[5]["mode_name"] == "ATT" and status[10]["goals_robot1"] == 1
    assert [s["theta"] for s in decoded["/robot1/state"]] == pytest.approx([theta(k) for k in range(N_STEPS)])


def test_tf_decodes_with_yaw_from_theta(shot_file):
    """Decode /tf with the protobuf schema stored in the file, as the Foxglove app does.
    Robot 1's frame faces the goal it attacks (x = 0): yaw theta + pi; the others use theta."""
    messages, _, _ = shot_file
    tf = [(schema, message) for topic, schema, message in messages if topic == "/tf"]
    schema = tf[0][0]
    assert schema.name == "foxglove.FrameTransforms" and schema.encoding == "protobuf"
    cls = protobuf_class(schema)

    for k, (_, message) in enumerate(tf):
        transforms = {t.child_frame_id: t for t in cls.FromString(message.data).transforms}
        assert set(transforms) == {"robot1", "robot2", "puck"}
        r1 = transforms["robot1"]
        assert r1.parent_frame_id == "world"
        assert r1.translation.x == pytest.approx(1.8 - 0.01 * k)
        yaw = theta(k) + np.pi
        assert (r1.rotation.z, r1.rotation.w) == pytest.approx((np.sin(yaw / 2), np.cos(yaw / 2)))
        assert r1.timestamp.seconds * 10**9 + r1.timestamp.nanos == message.log_time
        for body in ("robot2", "puck"):  # theta = 0 in the synthetic data: identity rotation
            rotation = transforms[body].rotation
            assert (rotation.z, rotation.w) == pytest.approx((0.0, 1.0))


def test_lines_well_above_table_and_candidates_drawn(shot_file):
    """Lines 3 mm above the table surface were hidden under it in the web app: keep a clear gap.
    A candidate gets one line per finite path (robot, shot); an unsolved one only its label."""
    messages, _, _ = shot_file
    scenes = [(topic, schema, message) for topic, schema, message in messages if schema.name == "foxglove.SceneUpdate"]
    cls = protobuf_class(scenes[0][1])
    entities = [(topic, entity) for topic, _, message in scenes for entity in cls.FromString(message.data).entities]
    surface = next(entity for topic, entity in entities if topic == "/scene/table").cubes[0]
    table_top = surface.pose.position.z + surface.size.z / 2

    lines_per_topic = {}
    for topic, entity in entities:
        for line in entity.lines:
            assert min(point.z for point in line.points) >= table_top + 0.01 - 1e-9, topic
        lines_per_topic.setdefault(topic, []).append(len(entity.lines))
    assert set(lines_per_topic["/shot_mpc/candidates/direct"]) == {1}  # robot path; shot is NaN in the data
    assert set(lines_per_topic["/shot_mpc/candidates/L"]) == {0}  # never valid


def test_table_cubes_opaque(shot_file):
    """One transparent cube makes the app draw the whole table without depth, over the lines on it."""
    messages, _, _ = shot_file
    schema, message = next((schema, message) for topic, schema, message in messages if topic == "/scene/table")
    cubes = protobuf_class(schema).FromString(message.data).entities[0].cubes
    assert len(cubes) == 3 and all(cube.color.a == 1.0 for cube in cubes)


def test_metadata_and_config_attachment(shot_file):
    _, metadata, attachments = shot_file
    assert metadata["holohover"]["scenario"] == "test"
    assert metadata["holohover"]["steps"] == str(N_STEPS)
    assert attachments == ["config.json"]


def test_run_sim_data_without_shot_mpc(config, tmp_path):
    """simulation.run_sim output: no candidates or log, both robots' inputs."""
    messages, _, _ = read(write_mcap(episode(config, shot=False), tmp_path / "sim.mcap"))
    topics = {topic for topic, _, _ in messages}
    assert "/robot2/control" in topics
    assert not any(topic.startswith("/shot_mpc") for topic in topics)
