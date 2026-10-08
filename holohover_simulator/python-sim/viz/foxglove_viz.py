"""
Foxglove export: writes an episode's recorded data (simulation.episode.episode_data) to an MCAP
file for the Foxglove web app.

    python main.py simulate --mcap --no-animate              # new run
    python main.py mcap simulation_data/<run>.pkl            # saved run

Open https://app.foxglove.dev, "Open local file..." with the .mcap, then import the layout
viz/foxglove_layout.json (README.md).

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
    /robot1/state, /robot2/state, /puck/state   JSON  x, y, theta, vx, vy, omega
    /robot1/control                  JSON             applied fx, fy, tau
    /status                          JSON             mode, time to impact, goals
    /events                          Log              mode switches, goals; contacts and the outcome (needs log)
  dial:
    /dial/samples                    SceneUpdate      robot paths of some samples of the last diffusion step
  shot_mpc:
    /robot1/plan_control, /robot1/def_control   JSON  plan and defence PD inputs before clipping
                                                      (plan: zero without one)
    /shot_mpc/candidates/<label>     SceneUpdate      robot path and shot of each bounce sequence
    /shot_mpc/scores                 JSON             score and validity of every sequence
    /shot_mpc/solves                 JSON             one message per selector update (needs log)
    /shot_mpc/compute/selector, /shot_mpc/compute/<label>   JSON  compute timeline (ComputeTimeline):
                                                      busy for the wall time of each update and
                                                      sequence solve, idle between (needs log)
    /events                                           also plan switches (needs log)
"""

import json
import math
import time
from pathlib import Path
from types import SimpleNamespace
from typing import get_args, get_origin

import numpy as np
from matplotlib.colors import to_rgba
from pydantic import BaseModel, ConfigDict, create_model

import foxglove
from foxglove import messages as fm
from foxglove.channels import FrameTransformsChannel, LogChannel, SceneUpdateChannel

MODE_NAMES = ("DEF", "ATT")
BODIES = ("robot1", "robot2", "puck")  # the 6-dim blocks of the 18-dim state, in order
# Frame yaw added to theta: x toward the goal each robot attacks. Robot 1 attacks x = 0, as in
# the shot MPC's planner frame (shot_mpc/params.py: theta_p = theta_w + pi); robot 2 attacks +x.
FRAME_YAW = {"robot1": math.pi, "robot2": 0.0, "puck": 0.0}
TRAIL_LENGTH = 60  # samples, as in the animation

# Heights (m). Overlays are 5 mm apart, stacked like in the animation, well above the table surface.
# What keeps lines visible is the table being opaque (table_scene), not the gap.
Z_TABLE, Z_MARKINGS = -0.02, -0.01  # table top; border, centre line and goals
Z_TRAIL, Z_CANDIDATE, Z_PREDICTION, Z_PLAN = 0.0, 0.005, 0.01, 0.015
Z_SAMPLE = Z_CANDIDATE
Z_BODY = 0.03  # disc height (from z = 0)

# Line widths in pixels; the candidates are faint except the executed one
LINE_PX = {"plan": 3.0, "prediction": 2.0, "trail": 2.0, "candidate": 2.0, "chosen": 4.0, "sample": 1.5}
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


def _color_over_table(name, alpha):
    """Opaque colour that looks like `name` at `alpha` over the table."""
    top, base = np.array(to_rgba(name)[:3]), np.array(to_rgba(TABLE_COLOR)[:3])
    r, g, b = alpha * top + (1.0 - alpha) * base
    return fm.Color(r=r, g=g, b=b, a=1.0)



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


#####################
### JSON messages ###
#####################
# Logged with model_dump_json, which converts numpy scalars and writes NaN and inf as null
# (the web app rejects NaN). model_dump keeps NaN: don't log its dict.
class BodyState(BaseModel):
    model_config = ConfigDict(title="holohover.BodyState")
    t: float
    x: float
    y: float
    theta: float
    vx: float
    vy: float
    omega: float


class Control(BaseModel):
    model_config = ConfigDict(title="holohover.Control")
    t: float
    fx: float
    fy: float
    tau: float


class Status(BaseModel):
    model_config = ConfigDict(title="holohover.Status")
    t: float
    mode: int
    mode_name: str
    time_to_impact: float | None  # ATT only
    goals_robot1: int
    goals_robot2: int


class ShotScores(BaseModel):
    """Score and validity of every bounce sequence, by label."""
    model_config = ConfigDict(title="holohover.ShotScores")
    t: float
    chosen: str | None  # None: no valid sequence
    scores: dict[str, float]
    valid: dict[str, bool]


class SolveResult(BaseModel):
    valid: bool
    cost: float
    score: float
    tf: float
    t_wall_ms: float
    iters: int
    cold: bool  # the solution kept is from a cold start
    start: str  # the solves run: warm, cold or warm+cold (the warm start failed)


class ShotSolve(BaseModel):
    """One selector update; results has the sequences it solved."""
    model_config = ConfigDict(title="holohover.ShotSolve")
    t: float
    outer: bool
    kind: str  # outer: every sequence, inner: B* only, commit: the committed B* near impact
    # No sequence solved: wall (the puck hits a wall before t_min) or reach (it stays out of our half)
    skip: str | None
    chosen: str | None
    t_total_ms: float  # the solve times added up
    t_wall_ms: float  # the update (shorter when solved in parallel)
    n_solves: int
    results: dict[str, SolveResult]


class ComputeState(BaseModel):
    """What one row of the compute timeline does from t on (ComputeTimeline)."""
    model_config = ConfigDict(title="holohover.ComputeState")
    t: float
    state: str  # idle, or while busy _selector_state or _sequence_state
    ms: float | None  # wall time of the busy interval; None when idle


def foxglove_schema(model):
    """JSON schema of `model` in plain types: $refs inlined, `X | None` written as X (the value may still
    be null). Lichtblick's parser (parseJsonSchema.ts) rejects $ref, anyOf and type lists; the Foxglove
    docs add $ref only."""
    schema = model.model_json_schema()
    defs = schema.pop("$defs", {})

    def simplify(node):
        if isinstance(node, list):
            return [simplify(v) for v in node]
        if not isinstance(node, dict):
            return node
        if "$ref" in node:
            return simplify(defs[node["$ref"].rsplit("/", 1)[-1]])
        options = [o for o in node.get("anyOf", ()) if o.get("type") != "null"]
        if len(options) == 1:
            return simplify(options[0])
        return {k: simplify(v) for k, v in node.items()}

    return simplify(schema)


def with_labels(model, labels):
    """`model` with each dict field written as one property per bounce-sequence label, so that the app
    offers them in message paths. For the channel schema only: the messages are the same."""
    fields = {}
    for name, field in model.model_fields.items():
        if get_origin(field.annotation) is dict:
            item = get_args(field.annotation)[1]
            per_label = create_model(f"{model.__name__}_{name}", **{label: (item | None, None) for label in labels})
            fields[name] = (per_label, ...)
    return create_model(model.__name__, __base__=model, **fields)


################
### Channels ###
################
def json_channel(ctx, topic, model):
    """JSON channel with `model`'s schema, named by its title."""
    return foxglove.Channel(topic, schema=foxglove_schema(model), context=ctx)


def setup_channels(ctx, labels=(), solves=False, robot1_controllers=False, samples=False):
    """All channels of one file, in context ctx; labels are the bounce sequences (may be empty).
    robot1_controllers: the plan and defence PD inputs of robot 1, besides the applied one.
    samples: DIAL's sample paths."""
    ch = {
        "tf": FrameTransformsChannel("/tf", context=ctx),
        "table": SceneUpdateChannel("/scene/table", context=ctx),
        "bodies": SceneUpdateChannel("/scene/bodies", context=ctx),
        "plan": SceneUpdateChannel("/robot1/plan", context=ctx),
        "puck_prediction": SceneUpdateChannel("/puck/prediction", context=ctx),
        "robot2_prediction": SceneUpdateChannel("/robot2/prediction", context=ctx),
        "trail": SceneUpdateChannel("/puck/trail", context=ctx),
        "status": json_channel(ctx, "/status", Status),
        "events": LogChannel("/events", context=ctx),
        "robot1_control": json_channel(ctx, "/robot1/control", Control),
    }
    for body in BODIES:
        ch[f"{body}_state"] = json_channel(ctx, f"/{body}/state", BodyState)
    if robot1_controllers:
        ch["robot1_plan_control"] = json_channel(ctx, "/robot1/plan_control", Control)
        ch["robot1_def_control"] = json_channel(ctx, "/robot1/def_control", Control)
    if len(labels):
        ch["scores"] = json_channel(ctx, "/shot_mpc/scores", with_labels(ShotScores, labels))
        ch["candidates"] = [SceneUpdateChannel(f"/shot_mpc/candidates/{name}", context=ctx) for name in labels]
    if samples:
        ch["samples"] = SceneUpdateChannel("/dial/samples", context=ctx)
    if solves:
        ch["solves"] = json_channel(ctx, "/shot_mpc/solves", with_labels(ShotSolve, labels))
        ch["compute"] = {row: json_channel(ctx, f"/shot_mpc/compute/{row}", ComputeState)
                         for row in ("selector", *labels)}
    return ch


####################
### Static scene ###
####################
def table_scene(config, stamp):
    """Table surface, border, goals (left red, right blue) and dashed centre line, as in the animation.
    Every cube is opaque: the app draws all cubes of an entity as transparent if one is, without
    writing depth, and the table surface then covers whatever lines happen to be drawn before it."""
    width, height = config["table"]["width"], config["table"]["height"]
    goal_width = config["table"]["goal_width"]
    goal_depth = width / 30
    goals = [
        fm.CubePrimitive(pose=_pose(x, height / 2, Z_MARKINGS - 0.001),
                         size=fm.Vector3(x=goal_depth, y=goal_width, z=0.002),
                         color=_color_over_table(color, 0.6))
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


def log_static(ch, config, ns):
    """Table and body discs, logged once."""
    stamp = _timestamp(ns)
    ch["table"].log(table_scene(config, stamp), log_time=ns)
    ch["bodies"].log(bodies_scene(config, stamp), log_time=ns)


##############
### Events ###
##############
def _events(data, log, dt):
    """(t, level, text) of mode switches and goals, plus contacts, the outcome and (shot_mpc) plan
    switches from log."""
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
    for t, name, tf in log.get("plan_history", ()):
        # A plan is re-accepted at every planning tick: report switches and the first plan after a contact
        if name != prev_label or any(prev_t < c <= t for c in contacts):
            events.append((t, Info, f"plan {name}: impact in {tf:.2f}s"))
        prev_t, prev_label = t, name
    t_end = (len(modes) - 1) * dt
    goal = f" at t={log['t_goal']:.2f}s" if log.get("t_goal") is not None else ""
    events.append((t_end, Info if log["outcome"] == "goal" else Warning, f"outcome: {log['outcome']}{goal}"))
    return events


def _kind(update):
    """Update kind; older logs have no commit."""
    return update.get("kind") or ("outer" if update["outer"] else "inner")


def _start(result):
    """Solves run for one sequence; older logs have no warm+cold."""
    return result.get("start") or ("cold" if result["cold"] else "warm")


def _selector_state(update):
    """Compute timeline state of an update: its kind, or "no shot (<skip>)" when it solved nothing."""
    if update["results"]:
        return _kind(update)
    return f"no shot ({update['skip']})" if update.get("skip") else "no shot"


def _sequence_state(result):
    """Compute timeline state of one sequence's solves: their start, then "fail" when none was valid."""
    return _start(result) + ("" if result["valid"] else " fail")


def solve_message(update):
    """ShotSolve of one selector update (controllers/shot_mpc/report.update_json)."""
    return ShotSolve(
        t=update["t"], outer=update["outer"], kind=_kind(update), skip=update.get("skip"), chosen=update["chosen"],
        t_total_ms=1e3 * update["t_total"], t_wall_ms=1e3 * update.get("t_wall", update["t_total"]),
        n_solves=len(update["results"]),
        results={name: SolveResult(valid=r["valid"], cost=r["cost"], score=r["score"], tf=r["tf"],
                                   t_wall_ms=1e3 * r["t_wall"], iters=r["iters"], cold=r["cold"], start=_start(r))
                 for name, r in update["results"].items()},
    )


class ComputeTimeline:
    """
    The selector updates (report.update_json) as busy intervals on the simulated-time axis, for a
    State Transitions panel: the "selector" row is busy (outer, inner, commit, or no shot when it
    solved nothing) for the wall time of each update, each sequence's row (warm, cold or warm+cold,
    plus fail when invalid) for the wall time of its solves, as if the simulation ran in real time;
    idle otherwise. Sequences solved in parallel start with the update, serial ones one after the
    other. An interval still running when the next one on its row starts is cut there: the
    simulation does not wait for the solver.
    add and flush return the messages to log, as (t, row, ComputeState).
    """

    def __init__(self):
        self.ends = {}  # row -> end of its busy interval, whose idle message is not out yet

    def add(self, update):
        """The messages of one update, the updates coming in time order."""
        items = []

        def busy(row, t, state, seconds):
            end = self.ends.pop(row, None)
            if end is not None and end < t:
                items.append((end, row, ComputeState(t=end, state="idle", ms=None)))
            items.append((t, row, ComputeState(t=t, state=state, ms=1e3 * seconds)))
            self.ends[row] = t + seconds

        t = update["t"]
        busy("selector", t, _selector_state(update), update.get("t_wall", update["t_total"]))
        for name, r in update["results"].items():
            busy(name, t, _sequence_state(r), r["t_wall"])
            if not update.get("parallel", False):
                t += r["t_wall"]
        return items

    def flush(self, t=np.inf):
        """The idle messages of the intervals ending before t, once no update can start before t."""
        done = sorted((end, row) for row, end in self.ends.items() if end < t)
        for _, row in done:
            del self.ends[row]
        return [(end, row, ComputeState(t=end, state="idle", ms=None)) for end, row in done]


##############
### Export ###
##############
def write_mcap(data, path, log=None, metadata=None, start_ns=None):
    """
    Write `data` (simulation.episode.episode_data) to the MCAP file `path`.
    log: optional simulation.episode.episode_log(result) (or the saved _log.json), which adds the
    contacts and the outcome, and for the shot MPC the selector updates and plan switches.
    metadata: extra key-values stored in the file (scenario, seed, ...).
    start_ns: wall-clock time of row 0 in ns (default: now).
    """
    config = data["config"]
    dt = 1.0 / config["simulation"]["hz"]
    start_ns = time.time_ns() if start_ns is None else int(start_ns)
    cand = data.get("candidates")
    samples = data.get("diffusion_trajs")
    run = SimpleNamespace(
        states=np.asarray(data["states"], dtype=float),
        inputs=np.asarray(data["input_sequence"], dtype=float),
        plan_inputs=data.get("plan_input_sequence"),
        def_inputs=data.get("def_input_sequence"),
        plans=np.asarray(data["path_plans"], dtype=float),
        tfs=np.asarray(data["tfs"], dtype=float),
        modes=np.asarray(data["modes"]),
        scores=np.asarray(data["scores"]),
        future=np.asarray(data["future_trajs"], dtype=float),
        samples=None if samples is None else np.asarray(samples)[:, -1],  # last diffusion step
        cand=cand,
        labels=list(cand["labels"]) if cand else [],
        r_robot=config["robot"]["radius"],
        r_puck=config["puck"]["radius"],
    )
    updates = (log or {}).get("updates", [])
    n = run.states.shape[0]

    # Timed extras (events, selector updates, compute timeline), logged in time order between the steps
    timed = [(t, 0, ("event", level, text)) for t, level, text in _events(data, log, dt)]
    timed += [(u["t"], 1, ("solve", u)) for u in updates]
    timeline = ComputeTimeline()
    compute = [item for u in updates for item in timeline.add(u)] + timeline.flush()
    timed += [(t, 2, ("compute", row, message)) for t, row, message in compute]
    timed.sort(key=lambda item: (item[0], item[1]))

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    ctx = foxglove.Context()  # one per file: several exports can run in one process
    writer = foxglove.open_mcap(str(path), allow_overwrite=True, context=ctx)
    try:
        ch = setup_channels(ctx, run.labels, solves=bool(updates), robot1_controllers=run.plan_inputs is not None,
                            samples=run.samples is not None)
        meta = {"source": "holohover python-sim", "controller": config.get("controller"), "dt": dt, "steps": n,
                **(metadata or {})}
        writer.write_metadata("holohover", {k: str(v) for k, v in meta.items()})
        writer.attach(log_time=start_ns, create_time=time.time_ns(), name="config.json",
                      media_type="application/json", data=json.dumps(config, indent=1, default=float).encode())

        log_static(ch, config, start_ns)

        i_timed = 0
        for k in range(n):
            t = k * dt
            while i_timed < len(timed) and timed[i_timed][0] <= t + 1e-9:
                _log_timed(ch, timed[i_timed], start_ns)
                i_timed += 1
            log_step(ch, run, k, t, start_ns + round(t * 1e9))
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
        ch["events"].log(fm.Log(timestamp=_timestamp(ns), level=level, message=text, name="holohover"), log_time=ns)
    elif kind == "compute":
        row, message = payload
        ch["compute"][row].log(message.model_dump_json(), log_time=ns)
    else:
        ch["solves"].log(solve_message(payload[0]).model_dump_json(), log_time=ns)


def log_step(ch, run, k, t, ns):
    """Everything shown for row k of the data: frames, states, inputs, status and overlays."""
    stamp = _timestamp(ns)
    state = run.states[k]

    transforms = []
    for j, body in enumerate(BODIES):
        x, y, theta, vx, vy, omega = state[6 * j:6 * j + 6]
        transforms.append(fm.FrameTransform(timestamp=stamp, parent_frame_id="world", child_frame_id=body,
                                            translation=fm.Vector3(x=x, y=y, z=0.0),
                                            rotation=_quat(theta + FRAME_YAW[body])))
        message = BodyState(t=t, x=x, y=y, theta=theta, vx=vx, vy=vy, omega=omega)
        ch[f"{body}_state"].log(message.model_dump_json(), log_time=ns)
    ch["tf"].log(fm.FrameTransforms(transforms=transforms), log_time=ns)

    controls = [("robot1_control", run.inputs[k])]
    if "robot1_plan_control" in ch:
        controls += [("robot1_plan_control", run.plan_inputs[k]), ("robot1_def_control", run.def_inputs[k])]
    for key, (fx, fy, tau) in controls:
        ch[key].log(Control(t=t, fx=fx, fy=fy, tau=tau).model_dump_json(), log_time=ns)

    mode = int(run.modes[k])
    ch["status"].log(Status(
        t=t, mode=mode, mode_name=MODE_NAMES[mode], time_to_impact=run.tfs[k] if mode == 1 else None,
        goals_robot1=run.scores[k, 0], goals_robot2=run.scores[k, 1],
    ).model_dump_json(), log_time=ns)

    def scene(channel, entity_id, lines):
        channel.log(fm.SceneUpdate(entities=[fm.SceneEntity(
            timestamp=stamp, frame_id="world", id=entity_id, lines=lines)]), log_time=ns)

    scene(ch["plan"], "plan", [_line(run.plans[k, :, :2], Z_PLAN, _color("red", 0.9), LINE_PX["plan"])])
    for key, cols, radius, color in (("puck_prediction", slice(12, 14), run.r_puck, "green"),
                                     ("robot2_prediction", slice(6, 8), run.r_robot, "#1f77b4")):
        xy = run.future[k, :, cols]
        scene(ch[key], key, [_line(xy, Z_PREDICTION, _color(color, 0.7), LINE_PX["prediction"]),
                             _circle(xy[-1], radius, Z_PREDICTION, _color(color, 0.7), LINE_PX["prediction"])])
    trail = run.states[max(0, k - TRAIL_LENGTH + 1):k + 1, 12:14]
    scene(ch["trail"], "trail", [_line(trail, Z_TRAIL, _color("gray", 0.55), LINE_PX["trail"])])

    if run.samples is not None:
        scene(ch["samples"], "samples", [_line(xy, Z_SAMPLE, _color("tab:purple", 0.4), LINE_PX["sample"])
                                         for xy in run.samples[k]])

    if not run.labels:
        return
    cand = {key: run.cand[key][k] for key in ("robot", "shot", "score", "valid", "chosen")}
    chosen = int(cand["chosen"])
    for j, name in enumerate(run.labels):
        style = "chosen" if j == chosen else "other"
        base = CANDIDATE_COLORS[j % len(CANDIDATE_COLORS)]
        color = _color(base, CANDIDATE_ALPHA[style])
        px = LINE_PX["chosen" if j == chosen else "candidate"]
        lines = [_line(xy, Z_CANDIDATE, color, px) for xy in (cand["robot"][j], cand["shot"][j])
                 if np.isfinite(xy).all(axis=1).any()]
        # The scores are plotted from /shot_mpc/scores rather than written on the table
        scene(ch["candidates"][j], name, lines)
    ch["scores"].log(ShotScores(
        t=t, chosen=run.labels[chosen] if chosen >= 0 else None,
        scores=dict(zip(run.labels, cand["score"])), valid=dict(zip(run.labels, cand["valid"])),
    ).model_dump_json(), log_time=ns)
