"""
The ROS nodes' topics in python-sim's MCAP, with their names, types and fields, so that one Foxglove
layout (holohover_utils/gui/air_hockey.foxglove.json) opens python-sim files and rosbags alike:

    /h0/state     holohover_msgs/msg/HolohoverStateStamped   robot 1 (h0), table frame
    /puck/state   holohover_msgs/msg/HolohoverStateStamped   the puck, table frame
    /h0/acc_ref   geometry_msgs/msg/AccelStamped             robot 1's input as an acceleration, table frame
    /h0/plan      holohover_msgs/msg/HolohoverPlan           shot_mpc: the planner's PlanSet at each update

The messages are CDR-encoded (mcap_ros2) with schemas in rosbag2's format, built from the .msg files
of holohover_msgs and the standard messages below: no ROS needed. Table frame: origin at the table
centre, the axes of python-sim's world frame (controllers/shot_mpc/frames.py).
"""

import re
from pathlib import Path

import foxglove
from mcap_ros2._dynamic import serialize_dynamic

MSG_DIR = Path(__file__).resolve().parents[3] / "holohover_firmware" / "extra_ros_packages" / "holohover_msgs" / "msg"
# The standard messages the topics use (comments left out)
STANDARD = {
    "std_msgs/Header": "builtin_interfaces/Time stamp\nstring frame_id\n",
    "builtin_interfaces/Time": "int32 sec\nuint32 nanosec\n",
    "geometry_msgs/Point": "float64 x\nfloat64 y\nfloat64 z\n",
    "geometry_msgs/Vector3": "float64 x\nfloat64 y\nfloat64 z\n",
    "geometry_msgs/Accel": "Vector3 linear\nVector3 angular\n",
    "geometry_msgs/AccelStamped": "std_msgs/Header header\nAccel accel\n",
}
PRIMITIVES = {"bool", "byte", "char", "float32", "float64", "int8", "uint8", "int16", "uint16", "int32", "uint32",
              "int64", "uint64", "string", "wstring"}
SEPARATOR = "=" * 80
TOPICS = {
    "/h0/state": "holohover_msgs/msg/HolohoverStateStamped",
    "/puck/state": "holohover_msgs/msg/HolohoverStateStamped",
    "/h0/acc_ref": "geometry_msgs/msg/AccelStamped",
    "/h0/plan": "holohover_msgs/msg/HolohoverPlan",
}


def _definition(name):
    """The .msg text of pkg/Name."""
    if name in STANDARD:
        return STANDARD[name]
    pkg, msg = name.split("/")
    assert pkg == "holohover_msgs", name
    return (MSG_DIR / f"{msg}.msg").read_text()


def _field_types(name):
    """The non-primitive field types of pkg/Name, as pkg/Name."""
    pkg = name.split("/")[0]
    out = []
    for line in _definition(name).splitlines():
        line = line.split("#")[0].strip()
        if not line or "=" in line.split()[-1]:  # blank, comment or constant
            continue
        base = re.sub(r"\[.*\]$", "", line.split()[0])
        base = re.sub(r"<=\d+$", "", base)
        if base in PRIMITIVES:
            continue
        out.append(base if "/" in base else f"{pkg}/{base}")
    return out


def schema_text(full_name):
    """The concatenated definition of pkg/msg/Name and its dependencies, as rosbag2 writes it."""
    pkg, _, msg = full_name.split("/")
    root, deps, stack = f"{pkg}/{msg}", [], []
    stack.extend(reversed(_field_types(root)))
    while stack:
        name = stack.pop()
        if name in deps:
            continue
        deps.append(name)
        stack.extend(reversed(_field_types(name)))
    return _definition(root) + "".join(f"\n{SEPARATOR}\nMSG: {name}\n{_definition(name)}" for name in deps)


class RosChannels:
    """The TOPICS as raw CDR channels of a foxglove context; log(topic, message dict, ns)."""

    def __init__(self, ctx):
        self.channels, self.encoders = {}, {}
        for topic, schema in TOPICS.items():
            text = schema_text(schema)
            self.encoders[topic] = serialize_dynamic(schema, text)[schema]
            self.channels[topic] = foxglove.Channel(
                topic, message_encoding="cdr", context=ctx,
                schema=foxglove.Schema(name=schema, encoding="ros2msg", data=text.encode()))

    def log(self, topic, message, ns):
        self.channels[topic].log(self.encoders[topic](message), log_time=ns)


def header(ns, frame_id="table"):
    return {"stamp": stamp(ns), "frame_id": frame_id}


def stamp(ns):
    ns = int(ns)
    return {"sec": ns // 10**9, "nanosec": ns % 10**9}


def state_message(ns, s):
    """HolohoverStateStamped from a table-frame state (x, y, theta, vx, vy, omega)."""
    x, y, theta, vx, vy, omega = (float(v) for v in s)
    return {"header": header(ns), "state_msg": {"x": x, "y": y, "v_x": vx, "v_y": vy, "yaw": theta, "w_z": omega}}


def accel_message(ns, a):
    """AccelStamped from an acceleration (a_x, a_y, a_theta), table frame."""
    return {"header": header(ns), "accel": {"linear": {"x": float(a[0]), "y": float(a[1]), "z": 0.0},
                                            "angular": {"x": 0.0, "y": 0.0, "z": float(a[2])}}}


def plan_message(ns, plans, start_ns):
    """HolohoverPlan from a holohover_planner PlanSet, as the shot_planner node publishes it (its
    plan_msg); the candidates' t0 are episode times, offset by the episode's start start_ns."""
    candidates = []
    for name, c in zip(plans.labels, plans.candidates):
        msg = {"label": name, "cost": float("nan"), "t0": stamp(0), "tf": 0.0, "states": [], "acc_x": [], "acc_y": [],
               "impact_puck": {"x": 0.0, "y": 0.0, "z": 0.0}, "puck_velocity": {"x": 0.0, "y": 0.0, "z": 0.0}}
        if c is not None:
            msg.update(
                cost=float(c.cost), t0=stamp(start_ns + round(c.t0 * 1e9)), tf=float(c.tf),
                states=[{"x": float(s[0]), "y": float(s[1]), "v_x": float(s[2]), "v_y": float(s[3]), "yaw": 0.0,
                         "w_z": 0.0} for s in c.states],
                acc_x=[float(a) for a in c.acc[:, 0]], acc_y=[float(a) for a in c.acc[:, 1]],
                impact_puck={"x": float(c.impact_puck[0]), "y": float(c.impact_puck[1]), "z": 0.0},
                puck_velocity={"x": float(c.puck_velocity[0]), "y": float(c.puck_velocity[1]), "z": 0.0},
            )
        candidates.append(msg)
    return {"header": header(ns), "planner": "shot_mpc", "chosen": int(plans.chosen),
            "label": plans.labels[plans.chosen] if plans.chosen >= 0 else "", "candidates": candidates}
