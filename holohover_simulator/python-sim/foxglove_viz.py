"""
Foxglove export: writes the dict that animation.TrajecoryAnimation takes (from
simulation.run_sim or run_shot.py) to an MCAP file for the Foxglove web app.

    python run_shot.py --scenario incoming --mcap --no-animate   # new run
    python foxglove_viz.py simulation_data/shot_incoming_<stamp>.pkl   # existing pickle

Open https://app.foxglove.dev, "Open local file..." with the .mcap, then import the
layout foxglove/shot_mpc_layout.json (see foxglove/README.md).

Frames: "world" is the simulator table frame (origin at the bottom-left corner, z up);
"robot1", "robot2" and "puck" are its children. Each robot frame's x-axis points at the
goal that robot attacks (FRAME_YAW); the /<body>/state topics keep the simulator's theta.
Row k of the data is logged at start + k / simulation.hz, the time the animation shows it at.

Topics:
    /tf                              FrameTransforms  world -> robot1, robot2, puck
    /scene/table, /scene/bodies      SceneUpdate      logged once; bodies follow their frame
    /robot1/plan                     SceneUpdate      planned robot 1 path (path_plans)
    /puck/prediction                 SceneUpdate      predicted puck path (future_trajs)
    /robot2/prediction               SceneUpdate      predicted robot 2 path (future_trajs)
    /puck/trail                      SceneUpdate      last TRAIL_LENGTH puck positions
    /shot_mpc/candidates/<label>     SceneUpdate      robot path and shot of each bounce sequence
    /robot1/state, /robot2/state, /puck/state   JSON  x, y, theta, vx, vy, omega
    /robot1/control (/robot2/control)           JSON  applied fx, fy, tau
    /status                          JSON             mode, time to impact, goals
    /shot_mpc/scores                 JSON             score and validity of every sequence
    /shot_mpc/solves                 JSON             one message per selector update (needs log)
    /events                          Log              mode switches, goals; contacts, plan
                                                      switches and the outcome (needs log)
"""

import argparse
import json
import math
import pickle
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from matplotlib.colors import to_rgba

import foxglove
from foxglove import messages as fm
from foxglove.channels import FrameTransformsChannel, LogChannel, SceneUpdateChannel

MODE_NAMES = ("DEF", "ATT")
BODIES = ("robot1", "robot2", "puck")  # the 6-dim blocks of the 18-dim state, in order
# Frame yaw added to theta: x toward the goal each robot attacks. Robot 1 attacks x = 0, as in
# the shot MPC's planner frame (shot_mpc/params.py: theta_p = theta_w + pi); robot 2 attacks +x.
FRAME_YAW = {"robot1": math.pi, "robot2": 0.0, "puck": 0.0}
TRAIL_LENGTH = 60  # samples, as in the animation

# Heights (m). The table surface sits well below everything drawn on it: lines 3 mm above it were
# hidden under it in the web app. Overlays are 5 mm apart, stacked like in the animation.
Z_TABLE, Z_MARKINGS = -0.02, -0.01  # table top; border, centre line and goals
Z_TRAIL, Z_CANDIDATE, Z_PREDICTION, Z_PLAN = 0.0, 0.005, 0.01, 0.015
Z_BODY, Z_TEXT = 0.03, 0.06  # disc height (from z = 0); labels

# Line widths in pixels; the candidates are faint except the executed one
LINE_PX = {"plan": 3.0, "prediction": 2.0, "trail": 2.0, "candidate": 2.0, "chosen": 4.0}
CANDIDATE_ALPHA = {"other": 0.3, "chosen": 0.85}
TABLE_COLOR = "#f7f7f7"
# Same colours as animation.TrajecoryAnimation.CANDIDATE_COLORS
CANDIDATE_COLORS = ["tab:blue", "tab:orange", "tab:purple", "tab:brown", "tab:pink",
                    "tab:olive", "tab:cyan", "tab:gray"]


#######################
### Message helpers ###
#######################
def _color(name, alpha=1.0):
    r, g, b, _ = to_rgba(name)
    return fm.Color(r=r, g=g, b=b, a=alpha)



def _timestamp(ns):
    return fm.Timestamp(ns // 1_000_000_000, ns % 1_000_000_000)


def _quat(yaw):
    return fm.Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def _pose(x=0.0, y=0.0, z=0.0, yaw=0.0):
    return fm.Pose(position=fm.Vector3(x=float(x), y=float(y), z=float(z)), orientation=_quat(float(yaw)))


def _points(xy, z):
    """Point3 list of a polyline; NaN rows (absent data) are dropped."""
    xy = np.asarray(xy, dtype=float)[:, :2]
    xy = xy[np.isfinite(xy).all(axis=1)]
    return [fm.Point3(x=float(x), y=float(y), z=z) for x, y in xy]


def _line(xy, z, color, px, kind=fm.LinePrimitiveLineType.LineStrip):
    return fm.LinePrimitive(type=kind, pose=_pose(), thickness=px, scale_invariant=True,
                            points=_points(xy, z), color=color)


def _circle(center, radius, z, color, px, n=32):
    a = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    xy = np.asarray(center, dtype=float)[:2] + radius * np.column_stack([np.cos(a), np.sin(a)])
    return _line(xy, z, color, px, fm.LinePrimitiveLineType.LineLoop)


def _text(x, y, text, color, size=14.0):
    return fm.TextPrimitive(pose=_pose(x, y, Z_TEXT), billboard=True, font_size=size,
                            scale_invariant=True, color=color, text=text)


def _json_ready(value):
    """Plain Python for json.dumps: numpy scalars converted, NaN/inf -> null (the web app rejects NaN)."""
    if isinstance(value, dict):
        return {str(k): _json_ready(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [_json_ready(v) for v in value]
    if isinstance(value, (bool, np.bool_)):
        return bool(value)
    if isinstance(value, (int, np.integer)):
        return int(value)
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else None
    return value


################
### Channels ###
################
NUMBER = {"type": ["number", "null"]}
BOOL = {"type": ["boolean", "null"]}
STRING = {"type": ["string", "null"]}

SOLVE_RESULT = {"type": "object", "properties": {
    "valid": BOOL, "cost": NUMBER, "score": NUMBER, "tf": NUMBER, "t_wall_ms": NUMBER, "iters": NUMBER, "cold": BOOL}}

JSON_SCHEMAS = {
    "holohover.BodyState": {k: NUMBER for k in ("t", "x", "y", "theta", "vx", "vy", "omega")},
    "holohover.Control": {k: NUMBER for k in ("t", "fx", "fy", "tau")},
    "holohover.Status": {"t": NUMBER, "mode": NUMBER, "mode_name": STRING, "time_to_impact": NUMBER,
                         "goals_robot1": NUMBER, "goals_robot2": NUMBER},
}


def _shot_schemas(labels):
    """Sequence labels as explicit properties, so Foxglove offers them in message paths."""
    def per_label(item):
        return {"type": "object", "properties": {name: item for name in labels}}

    return {
        "holohover.ShotScores": {"t": NUMBER, "chosen": STRING, "scores": per_label(NUMBER),
                                 "valid": per_label(BOOL)},
        "holohover.ShotSolve": {"t": NUMBER, "outer": BOOL, "chosen": STRING, "t_total_ms": NUMBER,
                                "n_solves": NUMBER, "results": per_label(SOLVE_RESULT)},
    }


def _json_channel(ctx, topic, schema_name, properties=None):
    schema = {"type": "object", "properties": properties or JSON_SCHEMAS[schema_name]}
    return foxglove.Channel(
        topic, message_encoding="json", context=ctx,
        schema=foxglove.Schema(name=schema_name, encoding="jsonschema", data=json.dumps(schema).encode()),
    )


def setup_channels(ctx, labels=(), robot2_control=False, solves=False):
    """All channels of one file, in context ctx; labels are the bounce sequences (may be empty)."""
    ch = {
        "tf": FrameTransformsChannel("/tf", context=ctx),
        "table": SceneUpdateChannel("/scene/table", context=ctx),
        "bodies": SceneUpdateChannel("/scene/bodies", context=ctx),
        "plan": SceneUpdateChannel("/robot1/plan", context=ctx),
        "puck_prediction": SceneUpdateChannel("/puck/prediction", context=ctx),
        "robot2_prediction": SceneUpdateChannel("/robot2/prediction", context=ctx),
        "trail": SceneUpdateChannel("/puck/trail", context=ctx),
        "status": _json_channel(ctx, "/status", "holohover.Status"),
        "events": LogChannel("/events", context=ctx),
        "robot1_control": _json_channel(ctx, "/robot1/control", "holohover.Control"),
    }
    for body in BODIES:
        ch[f"{body}_state"] = _json_channel(ctx, f"/{body}/state", "holohover.BodyState")
    if robot2_control:
        ch["robot2_control"] = _json_channel(ctx, "/robot2/control", "holohover.Control")
    shot = _shot_schemas(labels)
    if len(labels):
        ch["scores"] = _json_channel(ctx, "/shot_mpc/scores", "holohover.ShotScores", shot["holohover.ShotScores"])
        ch["candidates"] = [SceneUpdateChannel(f"/shot_mpc/candidates/{name}", context=ctx) for name in labels]
    if solves:
        ch["solves"] = _json_channel(ctx, "/shot_mpc/solves", "holohover.ShotSolve", shot["holohover.ShotSolve"])
    return ch


####################
### Static scene ###
####################
def table_scene(config, stamp):
    """Table surface, border, goals (left red, right blue) and dashed centre line, as in the animation."""
    width, height = config["table"]["width"], config["table"]["height"]
    goal_width = config["table"]["goal_width"]
    goal_depth = width / 30
    goals = [
        fm.CubePrimitive(pose=_pose(x, height / 2, Z_MARKINGS - 0.001),
                         size=fm.Vector3(x=goal_depth, y=goal_width, z=0.002),
                         color=_color(color, 0.6))
        for x, color in ((goal_depth / 2, "red"), (width - goal_depth / 2, "blue"))
    ]
    surface = fm.CubePrimitive(pose=_pose(width / 2, height / 2, Z_TABLE - 0.01),
                               size=fm.Vector3(x=width, y=height, z=0.02), color=_color(TABLE_COLOR))
    border = _line([[0, 0], [width, 0], [width, height], [0, height]], Z_MARKINGS, _color("black"), 3.0,
                   fm.LinePrimitiveLineType.LineLoop)
    dashes = np.linspace(0.0, height, 21)
    centre = _line([[width / 2, y] for y in dashes[:20]], Z_MARKINGS, _color("gray"), 2.0,
                   fm.LinePrimitiveLineType.LineList)
    return fm.SceneUpdate(entities=[fm.SceneEntity(
        timestamp=stamp, frame_id="world", id="table", cubes=[surface, *goals], lines=[border, centre],
    )])


def bodies_scene(config, stamp):
    """Robot and puck discs in their own frames (frame_locked: they move with /tf)."""
    r_robot, r_puck = config["robot"]["radius"], config["puck"]["radius"]
    entities = []
    for body, color in (("robot1", "black"), ("robot2", "darkblue")):
        disc = fm.CylinderPrimitive(pose=_pose(z=Z_BODY / 2), size=fm.Vector3(x=2 * r_robot, y=2 * r_robot, z=Z_BODY),
                                    bottom_scale=1.0, top_scale=1.0, color=_color(color, 0.6))
        heading = fm.SpherePrimitive(pose=_pose(0.6 * r_robot, 0.0, Z_BODY), color=_color("yellow"),
                                     size=fm.Vector3(x=0.16 * r_robot, y=0.16 * r_robot, z=0.16 * r_robot))
        entities.append(fm.SceneEntity(timestamp=stamp, frame_id=body, id=body, frame_locked=True,
                                       cylinders=[disc], spheres=[heading]))
    puck = fm.CylinderPrimitive(pose=_pose(z=Z_BODY / 4), size=fm.Vector3(x=2 * r_puck, y=2 * r_puck, z=Z_BODY / 2),
                                bottom_scale=1.0, top_scale=1.0, color=_color("green"))
    entities.append(fm.SceneEntity(timestamp=stamp, frame_id="puck", id="puck", frame_locked=True, cylinders=[puck]))
    return fm.SceneUpdate(entities=entities)


##############
### Events ###
##############
def _events(data, log, dt):
    """(t, level, text) of mode switches and goals, plus contacts, plan switches and the outcome from log."""
    Info, Warning = fm.LogLevel.Info, fm.LogLevel.Warning
    modes, scores = np.asarray(data["modes"]), np.asarray(data["scores"])
    events = []
    for k in range(1, len(modes)):
        if modes[k] != modes[k - 1]:
            events.append((k * dt, Info, f"mode {MODE_NAMES[int(modes[k])]}"))
        for robot in (0, 1):
            if scores[k, robot] > scores[k - 1, robot]:
                events.append((k * dt, Warning, f"goal for robot{robot + 1}: {scores[k, 0]}:{scores[k, 1]}"))
    if log is None:
        return events

    contacts = sorted(log["contacts"])
    events += [(t, Info, "contact robot1-puck") for t in contacts]
    prev_t, prev_label = -np.inf, None
    for t, name, tf in log["plan_history"]:
        # A plan is re-accepted at every planning tick: report switches and the first plan after a contact
        if name != prev_label or any(prev_t < c <= t for c in contacts):
            events.append((t, Info, f"plan {name}: impact in {tf:.2f}s"))
        prev_t, prev_label = t, name
    t_end = (len(modes) - 1) * dt
    goal = f" at t={log['t_goal']:.2f}s" if log.get("t_goal") is not None else ""
    events.append((t_end, Info if log["outcome"] == "goal" else Warning, f"outcome: {log['outcome']}{goal}"))
    return events


def _solve_message(update):
    return _json_ready({
        "t": update["t"], "outer": update["outer"], "chosen": update["chosen"],
        "t_total_ms": 1e3 * update["t_total"], "n_solves": len(update["results"]),
        "results": {name: {"valid": r["valid"], "cost": r["cost"], "score": r["score"], "tf": r["tf"],
                           "t_wall_ms": 1e3 * r["t_wall"], "iters": r["iters"], "cold": r["cold"]}
                    for name, r in update["results"].items()},
    })


##############
### Export ###
##############
def write_mcap(data, path, log=None, metadata=None, start_ns=None):
    """
    Write `data` (the dict TrajecoryAnimation takes) to the MCAP file `path`.
    log: optional run_shot.shot_log_json(result) (or the saved _log.json), which adds the
    selector updates, contacts, plan switches and the outcome.
    metadata: extra key-values stored in the file (scenario, seed, ...).
    start_ns: wall-clock time of row 0 in ns (default: now).
    """
    config = data["config"]
    dt = 1.0 / config["simulation"]["hz"]
    start_ns = time.time_ns() if start_ns is None else int(start_ns)
    cand = data.get("candidates")
    run = SimpleNamespace(
        states=np.asarray(data["states"], dtype=float),
        inputs=np.asarray(data["input_sequence"], dtype=float),
        plans=np.asarray(data["path_plans"], dtype=float),
        tfs=np.asarray(data["tfs"], dtype=float),
        modes=np.asarray(data["modes"]),
        scores=np.asarray(data["scores"]),
        future=np.asarray(data["future_trajs"], dtype=float),
        cand=cand,
        labels=list(cand["labels"]) if cand else [],
        r_robot=config["robot"]["radius"],
        r_puck=config["puck"]["radius"],
        table_width=config["table"]["width"],
    )
    n = run.states.shape[0]

    # Timed extras (events, selector updates), logged in time order between the steps
    timed = [(t, 0, ("event", level, text)) for t, level, text in _events(data, log, dt)]
    if log is not None:
        timed += [(u["t"], 1, ("solve", u)) for u in log["updates"]]
    timed.sort(key=lambda item: (item[0], item[1]))

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    ctx = foxglove.Context()  # one per file: several exports can run in one process
    writer = foxglove.open_mcap(str(path), allow_overwrite=True, context=ctx)
    try:
        ch = setup_channels(ctx, run.labels, robot2_control=run.inputs.shape[1] >= 6, solves=log is not None)
        meta = {"source": "holohover python-sim", "dt": dt, "steps": n, **(metadata or {})}
        writer.write_metadata("holohover", {k: str(v) for k, v in meta.items()})
        writer.attach(log_time=start_ns, create_time=time.time_ns(), name="config.json",
                      media_type="application/json", data=json.dumps(config, indent=1, default=float).encode())

        stamp0 = _timestamp(start_ns)
        ch["table"].log(table_scene(config, stamp0), log_time=start_ns)
        ch["bodies"].log(bodies_scene(config, stamp0), log_time=start_ns)

        i_timed = 0
        for k in range(n):
            t = k * dt
            while i_timed < len(timed) and timed[i_timed][0] <= t + 1e-9:
                _log_timed(ch, timed[i_timed], start_ns)
                i_timed += 1
            _log_step(ch, run, k, t, start_ns + round(t * 1e9))
        for item in timed[i_timed:]:
            _log_timed(ch, item, start_ns)
    finally:
        writer.close()
    return path


def _log_timed(ch, item, start_ns):
    t, _, (kind, *payload) = item
    ns = start_ns + round(t * 1e9)
    if kind == "event":
        level, text = payload
        ch["events"].log(fm.Log(timestamp=_timestamp(ns), level=level, message=text, name="holohover"),
                         log_time=ns)
    else:
        ch["solves"].log(_solve_message(payload[0]), log_time=ns)


def _log_step(ch, run, k, t, ns):
    """Everything shown at row k of the data: frames, states, inputs, status and overlays."""
    stamp = _timestamp(ns)

    transforms = []
    for j, body in enumerate(BODIES):
        x, y, theta, vx, vy, omega = run.states[k, 6 * j:6 * j + 6]
        transforms.append(fm.FrameTransform(timestamp=stamp, parent_frame_id="world", child_frame_id=body,
                                            translation=fm.Vector3(x=x, y=y, z=0.0),
                                            rotation=_quat(theta + FRAME_YAW[body])))
        ch[f"{body}_state"].log(_json_ready({"t": t, "x": x, "y": y, "theta": theta, "vx": vx, "vy": vy,
                                             "omega": omega}), log_time=ns)
    ch["tf"].log(fm.FrameTransforms(transforms=transforms), log_time=ns)

    for robot in (1, 2):
        if f"robot{robot}_control" in ch:
            fx, fy, tau = run.inputs[k, 3 * (robot - 1):3 * robot]
            ch[f"robot{robot}_control"].log(_json_ready({"t": t, "fx": fx, "fy": fy, "tau": tau}), log_time=ns)

    mode = int(run.modes[k])
    ch["status"].log(_json_ready({
        "t": t, "mode": mode, "mode_name": MODE_NAMES[mode], "time_to_impact": run.tfs[k] if mode == 1 else None,
        "goals_robot1": run.scores[k, 0], "goals_robot2": run.scores[k, 1],
    }), log_time=ns)

    def scene(channel, entity_id, lines, texts=()):
        channel.log(fm.SceneUpdate(entities=[fm.SceneEntity(
            timestamp=stamp, frame_id="world", id=entity_id, lines=lines, texts=list(texts))]), log_time=ns)

    scene(ch["plan"], "plan", [_line(run.plans[k, :, :2], Z_PLAN, _color("red", 0.9), LINE_PX["plan"])])
    for key, cols, radius, color in (("puck_prediction", slice(12, 14), run.r_puck, "green"),
                                     ("robot2_prediction", slice(6, 8), run.r_robot, "#1f77b4")):
        xy = run.future[k, :, cols]
        scene(ch[key], key, [_line(xy, Z_PREDICTION, _color(color, 0.7), LINE_PX["prediction"]),
                             _circle(xy[-1], radius, Z_PREDICTION, _color(color, 0.7), LINE_PX["prediction"])])
    trail = run.states[max(0, k - TRAIL_LENGTH + 1):k + 1, 12:14]
    scene(ch["trail"], "trail", [_line(trail, Z_TRAIL, _color("gray", 0.55), LINE_PX["trail"])])

    if not run.labels:
        return
    cand = run.cand
    chosen = int(cand["chosen"][k])
    for j, name in enumerate(run.labels):
        style = "chosen" if j == chosen else "other"
        base = CANDIDATE_COLORS[j % len(CANDIDATE_COLORS)]
        color = _color(base, CANDIDATE_ALPHA[style])
        px = LINE_PX["chosen" if j == chosen else "candidate"]
        lines = [_line(xy, Z_CANDIDATE, color, px) for xy in (cand["robot"][k, j], cand["shot"][k, j])
                 if np.isfinite(xy).all(axis=1).any()]
        score = cand["score"][k, j]
        status = "-" if not np.isfinite(score) else ("invalid" if not cand["valid"][k, j] else f"{score:.2f}")
        # Score row under the table, as in the animation
        label = _text(0.18 * j * run.table_width, -0.06, f"{'> ' if j == chosen else ''}{name} {status}",
                      _color(base))
        scene(ch["candidates"][j], name, lines, [label])
    ch["scores"].log(_json_ready({
        "t": t, "chosen": run.labels[chosen] if chosen >= 0 else None,
        "scores": dict(zip(run.labels, cand["score"][k])), "valid": dict(zip(run.labels, cand["valid"][k])),
    }), log_time=ns)


############
### Main ###
############
def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("pkl", help="simulation pickle (main.py or run_shot.py); its _log.json is used if present")
    parser.add_argument("--output", help="MCAP path (default: the pickle path with .mcap)")
    args = parser.parse_args()

    pkl = Path(args.pkl)
    with pkl.open("rb") as f:
        data = pickle.load(f)
    log_path = pkl.with_name(pkl.stem + "_log.json")
    log = json.loads(log_path.read_text()) if log_path.exists() else None
    output = write_mcap(data, args.output or pkl.with_suffix(".mcap"), log=log,
                        metadata={"source_file": pkl.name}, start_ns=int(pkl.stat().st_mtime * 1e9))
    print(f"Saved MCAP to {output}")


if __name__ == "__main__":
    main()
