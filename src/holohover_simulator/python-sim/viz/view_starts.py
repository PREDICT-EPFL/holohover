"""
The evaluation starts (config "evaluation.starts") in the Foxglove web app, with the episode layout
(viz/foxglove_layout.json). Run from python-sim/:

    python viz/view_starts.py                       # all starts (or python -m viz.view_starts)
    python viz/view_starts.py --first 37 --count 10
    python viz/view_starts.py --no-view             # only write simulation_data/eval_starts.mcap

The timeline shows one start per second (start first + k at t = k s). The table shows:
- the current start: robot 1, robot 2 (config "opponent") and the puck, with an arrow for the
  puck's first ARROW_S of travel; the Events panel gives its index, speed and heading (from +x,
  toward robot 1's goal);
- every start of the file's slice: robot 1 (black dots), stationary pucks (gray dots), moving
  pucks (blue dots, with a tick for their first TICK_S of travel).
"""

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

import foxglove
from foxglove import messages as fm

if not __package__:  # run as a file (python viz/view_starts.py): import from python-sim/, not viz/
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simulation.setup import load_config, load_starts, start_state
from viz.foxglove_open import open_in_foxglove
from viz.foxglove_viz import (Z_PREDICTION, Z_TRAIL, _color, _color_over_table, _line, _pose, _timestamp,
                              bodies_scene, log_bodies, setup_channels, table_scene)

PATH = Path("simulation_data/eval_starts.mcap")
PERIOD_NS = 1_000_000_000  # timeline time per start
ARROW_S, TICK_S = 0.5, 0.1  # puck travel (s) shown by the current start's arrow and the overview's ticks


def overview(starts, stamp):
    """Every start as dots on the table; opaque, as the app draws them in one mesh (table_scene)."""
    def dots(xy, color, d):
        return [fm.SpherePrimitive(pose=_pose(x, y, Z_TRAIL), size=fm.Vector3(x=d, y=d, z=d), color=color)
                for x, y in xy]

    r1 = np.array([s["robot1"] for s in starts])
    puck = np.array([s["puck"] for s in starts])
    moving = np.hypot(puck[:, 3], puck[:, 4]) > 0
    ticks = np.stack([puck[moving, :2], puck[moving, :2] + TICK_S * puck[moving, 3:5]], axis=1).reshape(-1, 2)
    spheres = (dots(r1[:, :2], _color("black"), 0.006)
               + dots(puck[~moving, :2], _color_over_table("gray", 0.6), 0.012)
               + dots(puck[moving, :2], _color_over_table("tab:blue", 0.6), 0.012))
    lines = [_line(ticks, Z_TRAIL, _color("tab:blue", 0.6), 1.5, fm.LinePrimitiveLineType.LineList)]
    return fm.SceneEntity(timestamp=stamp, frame_id="world", id="starts", spheres=spheres, lines=lines)


def log_start(ch, config, i, start, ns):
    """Bodies, puck arrow and Events line of start i."""
    stamp = _timestamp(ns)
    log_bodies(ch, start_state(config, start), float(i), ns)
    x, y, _, vx, vy, _ = start["puck"]
    speed, heading = math.hypot(vx, vy), math.atan2(vy, vx)
    arrows = []
    if speed > 0:
        length = ARROW_S * speed
        head = min(0.05, 0.4 * length)
        arrows.append(fm.ArrowPrimitive(pose=_pose(x, y, Z_PREDICTION, heading), shaft_length=length - head,
                                        shaft_diameter=0.01, head_length=head, head_diameter=0.03,
                                        color=_color("green")))
    ch["puck_prediction"].log(fm.SceneUpdate(entities=[fm.SceneEntity(
        timestamp=stamp, frame_id="world", id="puck_prediction", arrows=arrows)]), log_time=ns)
    motion = f"{speed:.2f} m/s at {math.degrees(heading):+.0f} deg" if speed > 0 else "stationary"
    ch["events"].log(fm.Log(timestamp=stamp, level=fm.LogLevel.Info, name="holohover",
                            message=f"start {i}: puck at ({x:.2f}, {y:.2f}), {motion}"), log_time=ns)


def write(config, starts, first, path=PATH):
    path.parent.mkdir(parents=True, exist_ok=True)
    ctx = foxglove.Context()
    writer = foxglove.open_mcap(str(path), allow_overwrite=True, context=ctx)
    try:
        ch = setup_channels(ctx)
        writer.write_metadata("holohover", {"source": "holohover python-sim", "starts": config["evaluation"]["starts"],
                                            "first": str(first), "count": str(len(starts))})
        start_ns = time.time_ns()
        stamp = _timestamp(start_ns)
        ch["table"].log(table_scene(config, stamp, extra=[overview(starts, stamp)]), log_time=start_ns)
        ch["bodies"].log(bodies_scene(config, stamp), log_time=start_ns)
        for k, start in enumerate(starts):
            log_start(ch, config, first + k, start, start_ns + k * PERIOD_NS)
    finally:
        writer.close()
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default="config/config.json")
    parser.add_argument("--first", type=int, default=0, help="index of the first start")
    parser.add_argument("--count", type=int, help="default: up to the last start")
    parser.add_argument("--no-view", action="store_true", help="don't open the file in Foxglove")
    args = parser.parse_args(argv)

    config = load_config(args.config)
    starts = load_starts(config["evaluation"]["starts"])[args.first:]
    starts = starts[:args.count] if args.count is not None else starts
    if not starts:
        parser.error("--first/--count: no start selected")
    speeds = np.array([math.hypot(*s["puck"][3:5]) for s in starts])
    moving = speeds[speeds > 0]
    print(f"Starts {args.first}..{args.first + len(starts) - 1}: {len(moving)} moving "
          f"({moving.min(initial=np.inf):.2f}-{moving.max(initial=0):.2f} m/s), {len(starts) - len(moving)} stationary")

    path = write(config, starts, args.first)
    print(f"Saved to {path} (start {args.first} + k at t = k s)")
    if not args.no_view:
        open_in_foxglove(path)


if __name__ == "__main__":
    main()
