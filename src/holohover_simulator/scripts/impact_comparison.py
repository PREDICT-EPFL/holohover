#!/usr/bin/env python3
"""Recorded wall impacts vs the simulator's three wall collision models, written as a bag for Foxglove.

Every wall_impact segment that the identification fitted (results.yaml next to the bag, from `identify.py segment`,
`review` and `fit` on the host) is replayed by collision_replay (the simulator's Box2D puck, walls and collision
code) from its fitted initial state, once per model: box2d, elastic and friction_restitution with the coefficients
of holohover_simulator/config/*.yaml. The free flight uses the fitted c_v, c_w and tilt and the fitted walls, so
the runs differ only by the collision model.

    ros2 run holohover_simulator impact_comparison.py /root/data/puck_id_2026-10-13_18-15-39

writes <bag>/impact_comparison/ (MCAP), at the recording's own times:
    /measured/puck/state                  EKF state of the recording, table frame (HolohoverStateStamped)
    /<model>/puck/state                   simulated state during each wall impact segment
    /<model>/position_error               |simulated - measured| position [mm] (std_msgs/Float64)
    /impact_comparison/scene              walls, pucks with a line turning with their spin, trails (MarkerArray)
    /impact_comparison/segment            segment, wall and errors of each impact, at its start (std_msgs/String)
    /impact_comparison/summary            errors over all impacts, at the start (std_msgs/String)

Open it in Foxglove (docker/data/... on the host) with the layout holohover_simulator/gui/impact_comparison.foxglove.json,
or play it live through the bridge: ros2 bag play <bag>/impact_comparison --loop --rate 0.25
"""
import argparse
import math
import os
import shutil
import subprocess
import sys

import numpy as np
import rosbag2_py
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from holohover_msgs.msg import HolohoverStateStamped
from rclpy.serialization import deserialize_message, serialize_message
from std_msgs.msg import ColorRGBA, Float64, String
from visualization_msgs.msg import Marker, MarkerArray

MODELS = ["box2d", "elastic", "friction_restitution"]
COLORS = {"box2d": (0.88, 0.31, 0.31), "elastic": (0.30, 0.56, 0.90), "friction_restitution": (0.30, 0.78, 0.39)}
MEASURED = (0.95, 0.95, 0.95)
TRAIL = 1.0  # [s] of measured trail


def stamp(t):
    ns = int(round(t * 1e9))
    return Time(sec=ns // 10**9, nanosec=ns % 10**9)


def bag_dir(path):
    path = os.path.abspath(path)
    return path if os.path.isdir(path) else os.path.dirname(path)


def read_states(path, topic):
    """Rows (t, x, y, yaw, vx, vy, w) of `topic`, sorted, repeated stamps dropped (as identification/data.py)."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    rows = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        m = deserialize_message(data, HolohoverStateStamped)
        s = m.state_msg
        rows.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, s.x, s.y, s.yaw, s.v_x, s.v_y, s.w_z])
    if not rows:
        sys.exit(f"no {topic} in {path}")
    rows = np.array(rows)
    rows = rows[np.argsort(rows[:, 0], kind="stable")]
    return rows[np.concatenate([[True], np.diff(rows[:, 0]) > 0])]


def to_table(rows, table):
    """Recording rows in the table frame of the identification."""
    tx, ty, tyaw = table
    c, s = math.cos(tyaw), math.sin(tyaw)
    out = rows.copy()
    dx, dy = rows[:, 1] - tx, rows[:, 2] - ty
    out[:, 1], out[:, 2] = c * dx + s * dy, -s * dx + c * dy
    out[:, 3] = np.angle(np.exp(1j * (rows[:, 3] - tyaw)))
    out[:, 4], out[:, 5] = c * rows[:, 4] + s * rows[:, 5], -s * rows[:, 4] + c * rows[:, 5]
    return out


def sim_coefficients():
    """Wall coefficients of each model from the simulator's config (box2d: its fixture values)."""
    config = os.path.join(get_package_share_directory("holohover_simulator"), "config")

    def params(name):
        with open(os.path.join(config, name)) as f:
            return yaml.safe_load(f)["/**"]["ros__parameters"]
    fr = params("friction_restitution.yaml")["friction_restitution"]["wall"]
    return {"box2d": {},
            "elastic": {"restitution": params("elastic.yaml")["elastic"]["wall"]["restitution"]},
            "friction_restitution": {k: fr[k] for k in ("e_n", "mu", "e_t")}}


def replay(tool, case):
    out = subprocess.run([tool], input=yaml.safe_dump(case), capture_output=True, text=True, check=True).stdout
    return np.loadtxt(out.splitlines()[1:], delimiter=",", ndmin=2)  # t, x, y, yaw, vx, vy, w


def state_msg(t, row):
    m = HolohoverStateStamped()
    m.header.stamp, m.header.frame_id = stamp(t), "table"
    s = m.state_msg
    s.x, s.y, s.yaw, s.v_x, s.v_y, s.w_z = (float(v) for v in row)
    return m


class Scene:
    """MarkerArrays of the walls and pucks, centred on the recorded motion (frame "world")."""

    def __init__(self, walls, rows, radius):
        self.radius = radius
        self.centre = (rows[:, 1:3].min(axis=0) + rows[:, 1:3].max(axis=0)) / 2
        self.walls = []
        for w in walls:  # wall faces: the puck centre touches at n . x = offset
            n = np.array(w["normal"], float)
            n /= np.linalg.norm(n)
            tv = np.array([-n[1], n[0]])
            along = rows[:, 1:3] @ tv
            p0 = (w["offset"] - radius) * n
            self.walls.append([p0 + (along.min() - 0.1) * tv, p0 + (along.max() + 0.1) * tv])

    def marker(self, t, ns, mid, mtype, rgb, alpha=1.0):
        m = Marker()
        m.header.stamp, m.header.frame_id = stamp(t), "world"
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        m.color = ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=float(alpha))
        return m

    def pt(self, xy, z=0.005):
        return Point(x=float(xy[0] - self.centre[0]), y=float(xy[1] - self.centre[1]), z=z)

    def line(self, t, ns, mid, points, rgb, width, z=0.005):
        m = self.marker(t, ns, mid, Marker.LINE_STRIP, rgb)
        m.scale.x = width
        m.points = [self.pt(p, z) for p in points]
        return m

    def puck(self, t, mid, row, rgb, label=None):
        r = self.radius
        disc = self.marker(t, "puck", mid, Marker.CYLINDER, rgb, 0.85)
        disc.pose.position = self.pt(row[1:3], 0.01)
        disc.scale.x = disc.scale.y = 2 * r
        disc.scale.z = 0.02
        x, y, yaw = row[1], row[2], row[3]
        out = [disc, self.line(t, "spin", mid, [(x, y), (x + r * math.cos(yaw), y + r * math.sin(yaw))],
                               (0.1, 0.1, 0.1), 0.006, z=0.025)]
        if label:
            text = self.marker(t, "label", mid, Marker.TEXT_VIEW_FACING, rgb)
            text.pose.position = self.pt((x, y + 0.06), 0.05)
            text.scale.z = 0.03
            text.text = label
            out.append(text)
        return out

    def frame(self, t, measured_trail, sims):
        """measured_trail: rows up to t; sims: {model: rows from the segment start up to t}"""
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers = [clear] + [self.line(t, "wall", i, w, (0.6, 0.6, 0.6), 0.01) for i, w in enumerate(self.walls)]
        markers.append(self.line(t, "trail", 0, measured_trail[:, 1:3], MEASURED, 0.004))
        markers += self.puck(t, 0, measured_trail[-1], MEASURED, "measured" if sims else None)
        for i, (name, rows) in enumerate(sims.items(), start=1):
            markers.append(self.line(t, "trail", i, rows[:, 1:3], COLORS[name], 0.004))
            markers += self.puck(t, i, rows[-1], COLORS[name], name)
        return MarkerArray(markers=markers)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="puck recording (directory or .mcap) with results.yaml next to it")
    ap.add_argument("--out", help="output bag directory (default: <bag>/impact_comparison)")
    args = ap.parse_args()

    in_dir = bag_dir(args.bag)
    res_file = os.path.join(in_dir, "results.yaml")
    if not os.path.exists(res_file):
        sys.exit(f"{res_file} not found: run identify.py segment, (review,) fit on the host first")
    with open(res_file) as f:
        res = yaml.safe_load(f)
    if "t_ref" not in res:
        sys.exit(f"{res_file} has no t_ref: it comes from an older identify.py, run `identify.py fit` again")
    t_ref, table = res["t_ref"], res["table_pose_in_mocap"]
    p = {k: v["value"] for k, v in res["parameters"].items()}
    radius = res["puck"]["radius"]
    walls = res["walls"]
    impacts = [g for g in res["segments"] if g["label"] == "wall_impact"]
    tool = os.path.join(get_package_prefix("holohover_simulator"), "lib", "holohover_simulator", "collision_replay")
    coeffs = sim_coefficients()

    rows = to_table(read_states(args.bag, res["config"]["topics"]["state"]), table)
    t = rows[:, 0] - t_ref
    print(f"{len(rows)} measured states, {len(impacts)} wall impacts, coefficients {coeffs}")

    # simulate every impact with every model
    sims, errors = [], []
    for g in impacts:
        idx = np.flatnonzero((t >= g["t0"]) & (t <= g["t1"]))
        run = {}
        for m in MODELS:
            case = {"model": m, **coeffs[m], "radius": radius, "state": [float(v) for v in g["state0"]],
                    "linear_damping": p["c_v"], "angular_damping": p["c_w"], "gravity": [p["g_x"], p["g_y"]],
                    "period": 0.01, "substeps": 10, "iterations": [6, 2], "duration": float(g["t1"] - g["t0"]),
                    "walls": [{"normal": w["normal"], "offset": w["offset"]} for w in walls]}
            r = replay(tool, case)
            r[:, 0] += g["t0"]
            run[m] = r
        # impact: first velocity jump of the simulation (the models agree until then)
        r = run[MODELS[0]]
        jump = np.flatnonzero(np.linalg.norm(np.diff(r[:, 4:6], axis=0), axis=1) > 0.1)
        t_imp = r[jump[0] + 1, 0] if jump.size else g["t0"]
        err = {}
        for m in MODELS:
            pos = np.column_stack([np.interp(t[idx], run[m][:, 0], run[m][:, k]) for k in (1, 2)])
            e = 1e3 * np.linalg.norm(pos - rows[idx, 1:3], axis=1)
            after = t[idx] > t_imp
            err[m] = (e, float(np.sqrt(np.mean(e[after] ** 2))) if after.any() else math.nan)
        sims.append((g, idx, run, err))
        errors.append([err[m][1] for m in MODELS])
        print(f"  segment {g['id']:3d}: " + "  ".join(f"{m} {err[m][1]:6.1f}" for m in MODELS) + "  mm after the impact")
    errors = np.array(errors)
    best = np.nanargmin(errors, axis=1)
    summary = [f"position RMS after the impact over {len(impacts)} wall impacts [mm]:"] + [
        f"  {m:22s} mean {np.nanmean(errors[:, k]):5.1f}  median {np.nanmedian(errors[:, k]):5.1f}  "
        f"best in {int(np.sum(best == k))}/{len(impacts)}" for k, m in enumerate(MODELS)]
    print("\n".join(summary))

    # messages, at the recording's times
    msgs = [(rows[0, 0], "/impact_comparison/summary", String(data="\n".join(summary)))]
    msgs += [(row[0], "/measured/puck/state", state_msg(row[0], row[1:])) for row in rows]
    active = {}  # measured index -> (segment, run)
    for g, idx, run, err in sims:
        for m in MODELS:
            msgs += [(t_ref + r[0], f"/{m}/puck/state", state_msg(t_ref + r[0], r[1:])) for r in run[m]]
        for m in MODELS:
            msgs += [(rows[i, 0], f"/{m}/position_error", Float64(data=float(e))) for i, e in zip(idx, err[m][0])]
        text = f"segment {g['id']}: " + ", ".join(f"{m} {err[m][1]:.1f} mm" for m in MODELS) + " (position RMS after the impact)"
        msgs.append((t_ref + g["t0"], "/impact_comparison/segment", String(data=text)))
        for i in idx:
            active[i] = run
    scene = Scene(walls, rows, radius)
    for i in range(len(rows)):
        trail = rows[np.searchsorted(rows[:, 0], rows[i, 0] - TRAIL):i + 1]
        sim_now = {m: r[r[:, 0] <= t[i] + 1e-9] for m, r in active.get(i, {}).items()}
        sim_now = {m: r for m, r in sim_now.items() if len(r)}
        msgs.append((rows[i, 0], "/impact_comparison/scene", scene.frame(rows[i, 0], trail, sim_now)))

    out = args.out or os.path.join(in_dir, "impact_comparison")
    if os.path.exists(out):
        shutil.rmtree(out)  # our own output of a previous run
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=out, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {}
    for _, topic, msg in msgs:
        if topic not in types:
            types[topic] = f"{type(msg).__module__.split('.')[0]}/msg/{type(msg).__name__}"
            writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=types[topic], serialization_format="cdr"))
    msgs.sort(key=lambda m: m[0])
    for tm, topic, msg in msgs:
        writer.write(topic, serialize_message(msg), int(round(tm * 1e9)))
    del writer
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
