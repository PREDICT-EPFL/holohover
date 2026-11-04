"""Loading the puck recording from a rosbag2 MCAP file and the smoothed velocity estimates."""
import math
import os
from dataclasses import dataclass, field

import numpy as np
import yaml
from scipy.signal import medfilt, savgol_filter

HERE = os.path.dirname(os.path.abspath(__file__))
CACHE_VERSION = 4


def load_config(path=None):
    """Default config.yaml, with an optional partial override file merged over it."""
    with open(os.path.join(HERE, "config.yaml")) as f:
        cfg = yaml.safe_load(f)
    if path:
        with open(path) as f:
            _merge(cfg, yaml.safe_load(f) or {})
    return cfg


def with_source(cfg, source):
    """Copy of cfg reading the given source (state or pose)."""
    return {**cfg, "source": source}


def load_for_segmentation(path, cfg):
    """Recording for segment/review: segmentation.source, or the fit source if the bag lacks it."""
    src = cfg["segmentation"]["source"]
    if src != cfg["source"]:
        try:
            return load(path, with_source(cfg, src))
        except RuntimeError as e:
            print(f"{e}: segmenting the {cfg['source']} data")
    return load(path, cfg)


def _merge(base, over):
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _merge(base[k], v)
        else:
            base[k] = v


def bag_paths(path):
    """(mcap file, output directory) for a bag directory or an .mcap file.

    Outputs (segments.yaml, results.yaml, plots) go next to the bag, or to
    identification/results/<bag name>/ when the bag directory is not writable (bags recorded
    in the docker container belong to root)."""
    path = os.path.abspath(path)
    if os.path.isdir(path):
        files = sorted(f for f in os.listdir(path) if f.endswith(".mcap"))
        if not files:
            raise FileNotFoundError(f"no .mcap file in {path}")
        if len(files) > 1:
            print(f"warning: {len(files)} .mcap files in {path}, using {files[0]}")
        mcap_file, bag_dir = os.path.join(path, files[0]), path
    else:
        mcap_file, bag_dir = path, os.path.dirname(path)
    if os.access(bag_dir, os.W_OK):
        return mcap_file, bag_dir
    out_dir = os.path.join(HERE, "results", os.path.basename(bag_dir.rstrip("/")))
    os.makedirs(out_dir, exist_ok=True)
    return mcap_file, out_dir


@dataclass
class Recording:
    """Puck samples in the table frame, times relative to t_ref (first puck stamp)."""
    t: np.ndarray          # (N,) [s]
    p: np.ndarray          # (N, 2) position [m]
    yaw: np.ndarray        # (N,) [rad], wrapped
    yaw_ok: np.ndarray     # (N,) bool, False where the yaw (and yaw rate) is not trusted
    t_ref: float           # absolute time of t = 0 (header stamp) [s]
    table: list            # table pose (x, y, yaw) used for the frame, in the mocap frame
    source: str
    out_dir: str
    kind: str = "state"    # "state": EKF state (velocities measured), "pose": raw mocap pose
    v_ekf: np.ndarray = None  # (N, 2) EKF velocity (state recordings)
    w_ekf: np.ndarray = None  # (N,) EKF yaw rate (state recordings)
    # filled by smooth()
    chunks: list = field(default_factory=list)  # (start, stop) index ranges without gaps
    v: np.ndarray = None   # (N, 2) velocity: EKF, or Savitzky-Golay of the pose
    a: np.ndarray = None   # (N, 2) acceleration (Savitzky-Golay of the position)
    yaw_u: np.ndarray = None  # (N,) unwrapped yaw, interpolated over invalid samples
    w: np.ndarray = None   # (N,) yaw rate: EKF, or Savitzky-Golay of the pose

    @property
    def measured_velocity(self):
        return self.kind == "state"

    @property
    def speed(self):
        return np.hypot(self.v[:, 0], self.v[:, 1])

    def state(self, i):
        return np.array([self.p[i, 0], self.p[i, 1], self.yaw_u[i], self.v[i, 0], self.v[i, 1], self.w[i]])

    def window(self, t0, t1):
        return np.flatnonzero((self.t >= t0) & (self.t <= t1))


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _read(mcap_file, cfg):
    """Rows (t, x, y, yaw, tilt, vx, vy, w) of the puck topic and of the table pose.

    tilt is |(qx, qy)| of the mocap orientation (0 for the EKF state), the velocities are nan
    for the raw pose."""
    from mcap_ros2.reader import read_ros2_messages

    topics = cfg["topics"]
    puck_topic = topics[cfg["source"]]
    puck, table = [], []
    for m in read_ros2_messages(mcap_file, topics=[puck_topic, topics["table"]]):
        s = m.ros_msg.header.stamp
        t = s.sec + s.nanosec * 1e-9
        if m.channel.topic == puck_topic and cfg["source"] == "state":
            x = m.ros_msg.state_msg
            puck.append([t, x.x, x.y, x.yaw, 0.0, x.v_x, x.v_y, x.w_z])
            continue
        pose = m.ros_msg.pose
        q = pose.orientation
        row = [t, pose.position.x, pose.position.y, _yaw(q), math.hypot(q.x, q.y), np.nan, np.nan, np.nan]
        (puck if m.channel.topic == puck_topic else table).append(row)
    if not puck:
        raise RuntimeError(f"no messages on {puck_topic} in {mcap_file} (source: {cfg['source']})")
    return np.array(puck), np.array(table) if table else None


def load(path, cfg):
    """Recording of the bag at `path` (directory or .mcap), cached next to the bag."""
    mcap_file, out_dir = bag_paths(path)
    cache = os.path.join(out_dir, f".identification_cache_{cfg['source']}.npz")
    key = f"{CACHE_VERSION}|{mcap_file}|{os.path.getmtime(mcap_file)}|{cfg['source']}|{cfg['topics']}"
    raw = table = None
    if os.path.exists(cache):
        c = np.load(cache, allow_pickle=False)
        if str(c["key"]) == key:
            raw, table = c["puck"], (c["table"] if c["table"].size else None)
    if raw is None:
        print(f"reading {mcap_file} ...")
        raw, table = _read(mcap_file, cfg)
        np.savez(cache, key=key, puck=raw, table=table if table is not None else np.empty(0))

    order = np.argsort(raw[:, 0], kind="stable")
    raw = raw[order]
    keep = np.concatenate([[True], np.diff(raw[:, 0]) > 0])  # drop repeated stamps
    raw = raw[keep]

    if table is not None:
        tx, ty = np.median(table[:, 1]), np.median(table[:, 2])
        tyaw = math.atan2(np.median(np.sin(table[:, 3])), np.median(np.cos(table[:, 3])))
    else:
        print(f"no {cfg['topics']['table']} in the bag: using the mocap frame")
        tx = ty = tyaw = 0.0
    c, s = math.cos(tyaw), math.sin(tyaw)
    dx, dy = raw[:, 1] - tx, raw[:, 2] - ty
    p = np.column_stack([c * dx + s * dy, -s * dx + c * dy])
    v = np.column_stack([c * raw[:, 5] + s * raw[:, 6], -s * raw[:, 5] + c * raw[:, 6]])
    yaw = np.angle(np.exp(1j * (raw[:, 3] - tyaw)))

    pre = cfg["preprocessing"]
    yaw_ok = raw[:, 4] <= pre["max_tilt"]
    if cfg["source"] == "state":
        # the EKF follows the mocap orientation flips: its yaw rate explodes there
        bad = np.abs(raw[:, 7]) > pre["max_yaw_rate"]
        yaw_ok &= ~(np.convolve(bad, np.ones(11), mode="same") > 0)
        # and its yaw is wrong where the raw orientation is invalid (when the bag has the pose)
        try:
            pose = load(path, with_source(cfg, "pose"))
        except RuntimeError:
            pose = None
        if pose is not None:
            t_bad = np.sort(pose.t_ref + pose.t[~pose.yaw_ok])
            if t_bad.size:
                i = np.searchsorted(t_bad, raw[:, 0])
                after = t_bad[np.minimum(i, t_bad.size - 1)] - raw[:, 0]
                before = raw[:, 0] - t_bad[np.maximum(i - 1, 0)]
                # one EKF sample and the next update (longer invalid stretches are covered anyway)
                yaw_ok &= ~((np.abs(after) < 0.015) | (np.abs(before) < 0.015))
    # running median of the unwrapped valid yaw catches flips the tilt check misses
    if yaw_ok.sum() > 10:
        idx = np.flatnonzero(yaw_ok)
        yu = np.unwrap(yaw[idx])
        med = medfilt(yu, 9)
        yaw_ok[idx[np.abs(yu - med) > pre["yaw_outlier"]]] = False

    rec = Recording(t=raw[:, 0] - raw[0, 0], p=p, yaw=yaw, yaw_ok=yaw_ok, t_ref=float(raw[0, 0]),
                    table=[float(tx), float(ty), float(tyaw)], source=mcap_file, out_dir=out_dir,
                    kind=cfg["source"], v_ekf=v, w_ekf=raw[:, 7].copy())
    smooth(rec, cfg)
    return rec


def _odd(n):
    n = int(n)
    return n if n % 2 else n + 1


def smooth(rec, cfg):
    """Velocities (EKF, or Savitzky-Golay for the raw pose) and Savitzky-Golay accelerations,
    per gap-free chunk."""
    pre = cfg["preprocessing"]
    n = rec.t.size
    breaks = np.flatnonzero(np.diff(rec.t) > pre["max_gap"]) + 1
    bounds = np.concatenate([[0], breaks, [n]])
    rec.chunks = [(int(a), int(b)) for a, b in zip(bounds[:-1], bounds[1:])]
    rec.v = np.full((n, 2), np.nan)
    rec.a = np.full((n, 2), np.nan)
    rec.w = np.full(n, np.nan)
    rec.yaw_u = np.full(n, np.nan)
    for a, b in rec.chunks:
        t = rec.t[a:b]
        ok = rec.yaw_ok[a:b]
        if ok.sum() >= 2:
            yu = np.unwrap(rec.yaw[a:b][ok])
            rec.yaw_u[a:b] = np.interp(t, t[ok], yu)
        else:
            rec.yaw_u[a:b] = rec.yaw[a:b]
        dt = np.median(np.diff(t)) if b - a > 1 else 1.0
        win = _odd(round(pre["smoothing_window"] / dt))
        if b - a < win + 2:
            continue
        for k in range(2):
            rec.a[a:b, k] = savgol_filter(rec.p[a:b, k], win, 3, deriv=2, delta=dt)
        if rec.measured_velocity:
            rec.v[a:b] = rec.v_ekf[a:b]
            rec.w[a:b] = rec.w_ekf[a:b]
        else:
            for k in range(2):
                rec.v[a:b, k] = savgol_filter(rec.p[a:b, k], win, 3, deriv=1, delta=dt)
            rec.w[a:b] = savgol_filter(rec.yaw_u[a:b], win, 3, deriv=1, delta=dt)
    # a chunk too short to smooth keeps nan velocities and is never used


def load_segments(path, rec):
    """segments.yaml with its times moved to the time base of `rec`.

    Times are relative to the first sample, which differs between the EKF state and the raw
    pose: segments made with one source still work with the other."""
    doc = load_yaml(path)
    for e in doc.get("events", []):
        if e["kind"] == "glitch":  # older name of the "flight" events
            e["kind"] = "flight"
    shift = float(doc.get("t_ref", rec.t_ref)) - rec.t_ref
    if abs(shift) > 1e-9:
        for g in doc.get("segments", []):
            g["t0"], g["t1"] = round(g["t0"] + shift, 4), round(g["t1"] + shift, 4)
        for e in doc.get("events", []):
            e["t"] = round(e["t"] + shift, 4)
        doc["t_ref"] = rec.t_ref
    return doc


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def save_yaml(path, data, header=""):
    with open(path, "w") as f:
        if header:
            f.write("".join(f"# {line}\n" for line in header.splitlines()))
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=None, width=120)
