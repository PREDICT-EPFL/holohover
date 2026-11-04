"""Synthetic bag with known parameters, to test the pipeline (segment -> fit recovers them).

The puck is simulated with the model in a box of walls, with a new random shot (a "hit")
every few seconds, sampled at 240 Hz with mocap-like noise, some flipped orientations and
one gap. Poses are written in a mocap frame rotated and shifted from the table frame, together
with the table pose, as rosbag2 MCAP. An EKF-like state (position, yaw, velocities with noise,
no filter dynamics) is written at 100 Hz on the state topic.
"""
import math
import os

import numpy as np

import model

TRUE = {"c_v": 0.03, "g_x": -0.012, "g_y": 0.008, "c_w": 0.05, "c_wc": 0.05, "e_n": 0.87, "mu": 0.08, "e_t": 0.45}
BOX = (0.45, 2.0, 0.0, 1.0)  # x_min, x_max, y_min, y_max of the puck centre in the table frame
TABLE = (3.7, 2.4, -math.pi / 2)  # table pose in the mocap frame
RATE = 240.0

STATE_STAMPED = """std_msgs/Header header
holohover_msgs/HolohoverState state_msg
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: holohover_msgs/HolohoverState
float64 x
float64 y
float64 v_x
float64 v_y
float64 yaw
float64 w_z
"""

POSE_STAMPED = """std_msgs/Header header
geometry_msgs/Pose pose
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: geometry_msgs/Pose
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
"""


def walls():
    x0, x1, y0, y1 = BOX
    return [model.make_wall("x_min", [1, 0], x0), model.make_wall("x_max", [-1, 0], -x1),
            model.make_wall("y_min", [0, 1], y0), model.make_wall("y_max", [0, -1], -y1)]


def trajectory(duration, cfg, seed=0, params=TRUE):
    """Times, states (table frame) and hit times of a sequence of random shots."""
    rng = np.random.default_rng(seed)
    r, k = cfg["puck"]["radius"], cfg["puck"]["kappa"]
    t = np.arange(0.0, duration, 1.0 / RATE)
    states = np.empty((t.size, 6))
    hits = []
    x0, x1, y0, y1 = BOX
    state = np.array([1.2, 0.5, 0.0, 0.0, 0.0, 0.0])
    t_shot, i = 0.0, 0
    while i < t.size:
        speed, angle = rng.uniform(0.5, 1.5), rng.uniform(-np.pi, np.pi)
        state[3:5] = speed * np.array([np.cos(angle), np.sin(angle)])
        state[5] = rng.uniform(-15.0, 15.0)
        length = rng.uniform(3.0, 6.0)
        j = int(np.searchsorted(t, t_shot + length))
        states[i:j] = model.rollout(state, t[i:j] - t_shot, params, walls(), r, k)
        if j >= t.size:
            break
        hits.append(t[j])
        t_shot = t[j]
        state = states[j - 1].copy()
        state[0] = np.clip(state[0], x0 + 0.05, x1 - 0.05)
        state[1] = np.clip(state[1], y0 + 0.05, y1 - 0.05)
        states[j - 1] = state
        i = j
    return t, states, hits


def write(out_dir, cfg, duration=60.0, seed=0, params=TRUE):
    from mcap_ros2.writer import Writer

    rng = np.random.default_rng(seed + 1)
    t, st, hits = trajectory(duration, cfg, seed, params)
    keep = ~((t > 20.0) & (t < 20.2))  # one gap
    t, st = t[keep], st[keep]
    tx, ty, tyaw = TABLE
    c, s = math.cos(tyaw), math.sin(tyaw)
    px = tx + c * st[:, 0] - s * st[:, 1] + rng.normal(0, 3e-4, t.size)
    py = ty + s * st[:, 0] + c * st[:, 1] + rng.normal(0, 3e-4, t.size)
    yaw = st[:, 2] + tyaw + rng.normal(0, 0.01, t.size)
    flipped = rng.random(t.size) < 0.01  # mocap orientation outliers
    t_ns0 = 1_700_000_000_000_000_000
    # EKF-like state at 100 Hz: every 2.4th pose sample
    state_idx = set(np.unique(np.round(np.arange(0, t.size, RATE / 100.0)).astype(int)).tolist())
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, os.path.basename(os.path.normpath(out_dir)) + "_0.mcap")
    with open(path, "wb") as f:
        w = Writer(f)
        schema = w.register_msgdef("geometry_msgs/msg/PoseStamped", POSE_STAMPED)
        state_schema = w.register_msgdef("holohover_msgs/msg/HolohoverStateStamped", STATE_STAMPED)

        def msg(stamp_ns, x, y, yaw_, tilt=False):
            q = (0.4, 0.0, math.sin(yaw_ / 2) * 0.9, math.cos(yaw_ / 2) * 0.9) if tilt else \
                (0.0, 0.0, math.sin(yaw_ / 2), math.cos(yaw_ / 2))
            return {"header": {"stamp": {"sec": stamp_ns // 1_000_000_000, "nanosec": stamp_ns % 1_000_000_000},
                               "frame_id": "world"},
                    "pose": {"position": {"x": x, "y": y, "z": 0.85},
                             "orientation": {"x": q[0], "y": q[1], "z": q[2], "w": q[3]}}}

        for k in range(t.size):
            ns = t_ns0 + int(round(t[k] * 1e9))
            w.write_message(cfg["topics"]["table"], schema, msg(ns, tx, ty, tyaw), log_time=ns + 7_000_000,
                            publish_time=ns + 7_000_000)
            yk = yaw[k] + (rng.uniform(1, 3) if flipped[k] else 0.0)
            w.write_message(cfg["topics"]["pose"], schema, msg(ns, px[k], py[k], yk, flipped[k]),
                            log_time=ns + 7_000_000, publish_time=ns + 7_000_000)
            if k in state_idx:
                vx = c * st[k, 3] - s * st[k, 4] + rng.normal(0, 0.01)
                vy = s * st[k, 3] + c * st[k, 4] + rng.normal(0, 0.01)
                w.write_message(cfg["topics"]["state"], state_schema,
                                {"header": {"stamp": {"sec": ns // 1_000_000_000, "nanosec": ns % 1_000_000_000},
                                            "frame_id": "world"},
                                 "state_msg": {"x": px[k], "y": py[k], "v_x": vx, "v_y": vy,
                                               "yaw": float(np.angle(np.exp(1j * yaw[k]))),
                                               "w_z": st[k, 5] + rng.normal(0, 0.1)}},
                                log_time=ns + 100_000, publish_time=ns + 100_000)
        w.finish()
    print(f"wrote {path}: {duration:.0f} s, {len(hits)} hits, parameters {params}")
    return path
