#!/usr/bin/env python3
"""
Evaluation of the air-hockey nodes (holohover_planner shot_planner, holohover_control control_plan) in
the Box2D simulation on python-sim's stored starts, like python-sim's `main.py evaluate`. With the
simulation launched for it:

    ros2 launch holohover_utils air_hockey_sim.launch.py evaluate:=50 [first:=0] [record:=true]

The launch file passes python-sim's directory (--python-sim, its launch argument python_sim).

An episode: /<name>/episode -1 idles the planner and the controller (the controller commands no
acceleration, without the disturbance compensation), the simulator is reset (/simulator/reset) to the start with the puck at
rest, and once h0's EKF has settled there it is reset again with the puck's velocity and the episode
starts (/<name>/episode <e>: the nodes restart afresh). It ends at the first goal or after python-sim's
evaluation.duration.

Goals as python-sim scores them (GoalDetector), contacts from the simulator (/hovercraft_collided).
Outcomes as python-sim's: goal (first shot: one contact before the goal), late_goal (more contacts, not
counted), own_goal, miss (contact, no goal), no_contact (a plan was chosen, no contact), no_attack.

Timing, the real-time check: the period of /<name>/acc_ref (control_plan, one per control step) and the
delay of each new plan followed, from its t0 to its first arrival here (the solve time plus the transport).

Results: one JSON file, also when interrupted (Ctrl-C), in /root/data/box2d_eval (docker/data on the host).
"""

import argparse
import json
import os
import sys
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from holohover_msgs.msg import HolohoverPlan, HolohoverStateStamped
from geometry_msgs.msg import AccelStamped
from std_msgs.msg import Bool, Float64MultiArray, Int32

OUT_DIR = "/root/data/box2d_eval"
SETTLE_POS, SETTLE_VEL = 0.005, 0.02  # m, m/s: h0's EKF state is at the start, at rest
SETTLE_MIN, SETTLE_MAX = 0.3, 3.0  # s
STOP_WAIT = 0.3  # s from idling the nodes to the reset: the motors settle (time constant 0.03 s)
READY_TIMEOUT = 60.0  # s to wait for the simulation and the nodes
CONTACT_MERGE = 0.05  # s: /hovercraft_collided messages closer than this are one contact


def stamp_s(stamp):
    return stamp.sec + 1e-9 * stamp.nanosec


class GoalDetector:
    """Goals as python-sim scores them: the puck reaches an end wall inside the goal mouth. The Box2D end
    walls are closed, so the puck bounces off the goal line: a goal is a reversal of the puck's x velocity
    with its centre within r_puck + reach of an end wall, inside the mouth. Table frame (origin at the
    centre); h0 attacks the left goal (x = -width / 2)."""

    def __init__(self, width, goal_width, r_puck, reach=0.05):
        self.x_wall, self.y_mouth, self.x_reach = 0.5 * width, 0.5 * goal_width, r_puck + reach
        self.vx = None

    def update(self, x, y, vx):
        """"goal" (left goal), "own_goal" (right goal) or None."""
        vx_prev, self.vx = self.vx, vx
        if vx_prev is None or abs(y) > self.y_mouth:
            return None
        if vx_prev < 0.0 <= vx and x < -self.x_wall + self.x_reach:
            return "goal"
        if vx_prev > 0.0 >= vx and x > self.x_wall - self.x_reach:
            return "own_goal"
        return None


def outcome(goal, contacts, attacked):
    if goal is not None:
        return goal if goal == "own_goal" or len(contacts) == 1 else "late_goal"
    if contacts:
        return "miss"
    return "no_contact" if attacked else "no_attack"


def stats_ms(values):
    a = 1e3 * np.asarray(values, dtype=float)
    if len(a) == 0:
        return None
    return {"n": len(a), "mean": float(a.mean()), "p95": float(np.percentile(a, 95)), "max": float(a.max())}


class Evaluator(Node):
    def __init__(self, name, table):
        super().__init__("air_hockey_evaluate")
        self.reset_publisher = self.create_publisher(Float64MultiArray, "/simulator/reset", 10)
        self.episode_publisher = self.create_publisher(Int32, f"/{name}/episode", 10)
        self.create_subscription(HolohoverStateStamped, f"/{name}/state", self.state_callback, 10)
        self.create_subscription(HolohoverStateStamped, "/puck/state", self.puck_callback, 10)
        self.create_subscription(Bool, "/hovercraft_collided", self.contact_callback, 10)
        self.create_subscription(HolohoverPlan, f"/{name}/plan", self.plan_callback, 10)
        self.create_subscription(AccelStamped, f"/{name}/acc_ref", self.acc_callback, 100)
        self.name, self.table = name, table
        self.state = None  # h0 (x, y, v_x, v_y)
        self.running = False  # an episode is on: goals, contacts, plans and timing are recorded
        self.t_start = 0.0
        self.goals, self.contacts, self.attacked = None, [], False
        self.detector = None
        self.acc_times, self.plan_delays = [], []  # this episode's
        self.t0_followed = None  # t0 of the plan followed (republished while its re-solves fail)

    def now_s(self):
        return 1e-9 * self.get_clock().now().nanoseconds

    def state_callback(self, msg):
        s = msg.state_msg
        self.state = np.array([s.x, s.y, s.v_x, s.v_y])

    def puck_callback(self, msg):
        if not self.running or self.goals is not None:
            return
        s = msg.state_msg
        goal = self.detector.update(s.x, s.y, s.v_x)
        if goal is not None:
            self.goals = (goal, self.now_s() - self.t_start)

    def contact_callback(self, msg):
        t = self.now_s() - self.t_start
        if msg.data and self.running and (not self.contacts or t - self.contacts[-1] > CONTACT_MERGE):
            self.contacts.append(t)

    def plan_callback(self, msg):
        if self.running and msg.chosen >= 0:
            self.attacked = True
            t0 = stamp_s(msg.candidates[msg.chosen].t0)
            if t0 != self.t0_followed:
                self.t0_followed = t0
                self.plan_delays.append(self.now_s() - t0)

    def acc_callback(self, msg):
        if self.running:
            self.acc_times.append(self.now_s())

    def wait(self, condition, timeout):
        """Spin until condition() or the timeout (s); returns condition()."""
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end and not condition():
            rclpy.spin_once(self, timeout_sec=0.01)
        return condition()

    def reset(self, robot, puck):
        self.reset_publisher.publish(Float64MultiArray(data=[float(v) for v in [*robot, *puck]]))

    def episode(self, e, robot, puck, duration):
        """One episode from robot (x, y, yaw, v_x, v_y, w_z) and puck (x, y, v_x, v_y), table frame."""
        self.episode_publisher.publish(Int32(data=-1))
        self.wait(lambda: False, STOP_WAIT)
        self.reset(robot, [puck[0], puck[1], 0.0, 0.0])
        t_reset = time.monotonic()
        settled = bool(self.wait(lambda: time.monotonic() - t_reset > SETTLE_MIN and self.state is not None
                                 and np.linalg.norm(self.state[0:2] - robot[0:2]) < SETTLE_POS
                                 and np.linalg.norm(self.state[2:4]) < SETTLE_VEL, SETTLE_MAX))
        if not settled:
            err, speed = np.linalg.norm(self.state[0:2] - robot[0:2]), np.linalg.norm(self.state[2:4])
            self.get_logger().warn(f"episode {e}: h0's EKF did not settle at the start ({1e3 * err:.1f} mm, "
                                   f"{speed:.3f} m/s), running it anyway")

        self.goals, self.contacts, self.attacked = None, [], False
        self.acc_times, self.plan_delays, self.t0_followed = [], [], None
        self.detector = GoalDetector(*self.table)
        self.reset(robot, puck)
        self.t_start, self.running = self.now_s(), True
        self.episode_publisher.publish(Int32(data=e))
        self.wait(lambda: self.goals is not None, duration)
        self.running = False
        self.episode_publisher.publish(Int32(data=-1))

        goal, t_goal = self.goals if self.goals is not None else (None, None)
        contacts = [t for t in self.contacts if t_goal is None or t <= t_goal]
        res = outcome(goal, contacts, self.attacked)
        return {"outcome": res, "t_goal": t_goal, "contacts": contacts, "first_shot": res == "goal",
                "settled": settled, "acc_ref_periods": list(np.diff(self.acc_times)),
                "plan_delays": self.plan_delays}


def main(argv=None):
    argv = sys.argv if argv is None else argv
    p = argparse.ArgumentParser(description="Evaluate the air-hockey nodes in Box2D on python-sim's starts")
    p.add_argument("--python-sim", required=True,
                   help="python-sim's directory: its config.json (evaluation settings, table) and stored starts")
    p.add_argument("--first", type=int, default=0, help="first start")
    p.add_argument("--episodes", type=int, default=50, help="number of starts")
    p.add_argument("--name", default="h0", help="hovercraft (namespace of the planner and the controller)")
    p.add_argument("--out", help=f"JSON results (default: {OUT_DIR}/air_hockey_eval_<stamp>.json)")
    args = p.parse_args(rclpy.utilities.remove_ros_args(argv)[1:])

    with open(os.path.join(args.python_sim, "config", "config.json")) as f:
        config = json.load(f)
    with open(os.path.join(args.python_sim, config["evaluation"]["starts"])) as f:
        starts = json.load(f)["starts"][args.first:args.first + args.episodes]
    duration = config["evaluation"]["duration"]
    tbl = config["table"]
    offset = np.array([0.5 * tbl["width"], 0.5 * tbl["height"]])  # python-sim world -> table frame

    rclpy.init(args=argv)
    node = Evaluator(args.name, (tbl["width"], tbl["goal_width"], config["puck"]["radius"]))
    print(f"Waiting for the simulation and /{args.name}'s planner and controller (at most {READY_TIMEOUT:.0f} s)",
          flush=True)
    if not node.wait(lambda: node.count_subscribers("/simulator/reset") > 0
                     and node.count_subscribers(f"/{args.name}/episode") >= 2 and node.state is not None,
                     READY_TIMEOUT):
        print(f"No simulation: /simulator/reset, the subscribers of /{args.name}/episode or /{args.name}/state "
              "missing. Run: ros2 launch holohover_utils air_hockey_sim.launch.py evaluate:=50", file=sys.stderr)
        node.destroy_node()
        rclpy.try_shutdown()
        return 1

    results, counts = [], {}
    try:
        for e, start in enumerate(starts, args.first):
            robot = np.array(start["robot1"], dtype=float)  # python-sim world frame
            puck = np.array(start["puck"], dtype=float)
            robot[0:2] -= offset
            puck[0:2] -= offset
            res = node.episode(e, robot, puck[[0, 1, 3, 4]], duration)
            results.append({"episode": e, **res})
            counts[res["outcome"]] = counts.get(res["outcome"], 0) + 1
            print(f"episode {e:3d}: {res['outcome']:<10} contacts={len(res['contacts'])}", flush=True)
    except KeyboardInterrupt:
        print(f"\nInterrupted after {len(results)} episodes")
    finally:
        try:
            node.episode_publisher.publish(Int32(data=-1))
        except Exception:
            pass
        out = args.out or os.path.join(OUT_DIR if os.path.isdir(os.path.dirname(OUT_DIR)) else ".",
                                       f"air_hockey_eval_{datetime.now():%Y%m%d_%H%M%S}.json")
        os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
        with open(out, "w") as f:
            json.dump({"first": args.first, "duration": duration, "results": results}, f, indent=1)
        node.destroy_node()
        rclpy.try_shutdown()

    n, goals = len(results), counts.get("goal", 0)
    print(f"\nConversion: {goals}/{n} = {100 * goals / max(n, 1):.0f}% (first-shot goals) | outcomes: {counts}")
    periods = stats_ms([d for r in results for d in r["acc_ref_periods"]])
    delays = stats_ms([d for r in results for d in r["plan_delays"]])
    if periods:
        late = sum(d > 0.015 for r in results for d in r["acc_ref_periods"])
        print(f"acc_ref period (control_plan, 10 ms): n={periods['n']} mean={periods['mean']:.2f} "
              f"p95={periods['p95']:.2f} max={periods['max']:.2f} ms, {late} over 15 ms")
    if delays:
        print(f"plan delay (t0 -> arrival): n={delays['n']} mean={delays['mean']:.1f} p95={delays['p95']:.1f} "
              f"max={delays['max']:.1f} ms")
    print(f"Saved to {os.path.abspath(out)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
