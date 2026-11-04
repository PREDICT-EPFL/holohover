"""
ROS 2 node of the shot MPC planner for one hovercraft (namespace hN). The planning itself is
shot_planner.ShotPlanner, which python-sim runs too; this node feeds it and publishes its plans.

    state         (in)   HolohoverStateStamped, the hovercraft's EKF (holohover_navigation), table frame
    /puck/state   (in)   HolohoverStateStamped, the puck's EKF (a navigation node in namespace puck)
    episode       (in)   std_msgs/Int32 of an evaluation (holohover_utils air_hockey_evaluate.py): -1 idles
                         the planner, an episode index restarts it afresh (parameter start_idle: idle at start)
    plan          (out)  HolohoverPlan, table frame, after every planning update: one candidate per bounce
                         sequence (its latest valid plan with its cost, empty if none) and the index of
                         the one to follow (chosen, -1: none); every candidate empty and chosen = -1 after a
                         contact or an expired plan

A timer at shot_mpc.rates.inner_hz runs the planner's ticks. A plan is published once solved, so the
controller gets it a solve time after its t0 and follows it on the clock.

Contacts are detected with the planner's puck model (contact.py), in the puck state callback.

The last input fed to the NLP (u_prev, cost term w_du) is the current plan's input at t0, zero
without a plan: the input actually applied by the controller is not known here.
"""

import time

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor
from holohover_msgs.msg import HolohoverPlan, HolohoverPlanCandidate, HolohoverState, HolohoverStateStamped
from std_msgs.msg import Int32

from .contact import ContactDetector
from .shot_mpc.params import PARAMETERS, Frame, ShotParams
from .shot_mpc.report import timing_summary, update_json
from .shot_mpc.shot_nlp import ShotNLP
from .shot_planner import Body, ShotPlanner

REPORT_PERIOD = 5.0  # s between two timing reports
# The node's own parameters, besides ShotParams' (PARAMETERS); start_idle is the launch file's (default false)
NODE_PARAMETERS = ("attack_goal", "max_state_age", "shot_mpc.n_workers", "contact.dv", "contact.margin",
                   "contact.lookback")


def stamp_s(stamp):
    return stamp.sec + 1e-9 * stamp.nanosec


def body(msg):
    """A HolohoverStateStamped as a Body (table frame)."""
    s = msg.state_msg
    return Body(stamp_s(msg.header.stamp), s.x, s.y, s.v_x, s.v_y)


def candidate_msg(name, c):
    """One HolohoverPlanCandidate from a Candidate, empty (cost NaN) for None."""
    msg = HolohoverPlanCandidate(label=name, cost=float("nan"))
    if c is None:
        return msg
    msg.cost = c.cost
    msg.t0 = Time(nanoseconds=int(round(c.t0 * 1e9))).to_msg()
    msg.tf = c.tf
    msg.states = [HolohoverState(x=float(s[0]), y=float(s[1]), v_x=float(s[2]), v_y=float(s[3])) for s in c.states]
    msg.acc_x = c.acc[:, 0].tolist()
    msg.acc_y = c.acc[:, 1].tolist()
    msg.impact_puck.x, msg.impact_puck.y = float(c.impact_puck[0]), float(c.impact_puck[1])
    msg.puck_velocity.x, msg.puck_velocity.y = float(c.puck_velocity[0]), float(c.puck_velocity[1])
    return msg


def plan_msg(stamp, plans):
    """A HolohoverPlan from a PlanSet."""
    msg = HolohoverPlan()
    msg.header.stamp = stamp
    msg.header.frame_id = "table"
    msg.planner = "shot_mpc"
    msg.chosen = plans.chosen
    msg.label = plans.labels[plans.chosen] if plans.chosen >= 0 else ""
    msg.candidates = [candidate_msg(name, c) for name, c in zip(plans.labels, plans.candidates)]
    return msg


class ShotPlannerNode(Node):
    def __init__(self):
        super().__init__("shot_planner")
        p = self.parameters(PARAMETERS + NODE_PARAMETERS)
        self.P = P = ShotParams.from_parameters(p)
        frame = Frame(P, p["attack_goal"])
        n_workers = p["shot_mpc.n_workers"]  # 0 or 1: serial solves
        max_age = p["max_state_age"]  # s: older states are not planned from
        self.contacts = ContactDetector(P, dv=p["contact.dv"], margin=p["contact.margin"],
                                        lookback=p["contact.lookback"])

        t_build = time.perf_counter()
        self.nlp = ShotNLP(P, verbose=False)
        pool = self.nlp.start_pool(n_workers)
        self.get_logger().info(
            f"shot NLP: {self.nlp.nw} variables, built in {time.perf_counter() - t_build:.1f} s, "
            + (f"{pool.n_workers} solver processes" if pool else "serial solves"))
        self.planner = ShotPlanner(P, frame, self.nlp, max_state_age=max_age, log=self.get_logger().info)
        self.active = not self.declare_parameter("start_idle", False).value  # idle: no planning (episode -1)

        self.robot = None  # latest Body of the hovercraft
        self.puck = None  # latest Body of the puck
        self.updates = []  # selector updates since the last report (update_json)
        self.overruns = 0  # ticks that took longer than their period
        self.t_report = None

        self.plan_publisher = self.create_publisher(HolohoverPlan, "plan", 10)
        # The states stay fresh while a solve blocks the planning timer
        sensors = MutuallyExclusiveCallbackGroup()
        self.create_subscription(HolohoverStateStamped, "state", self.state_callback, 10, callback_group=sensors)
        self.create_subscription(HolohoverStateStamped, "/puck/state", self.puck_callback, 10,
                                 callback_group=sensors)
        self.create_subscription(Int32, "episode", self.episode_callback, 10, callback_group=sensors)
        self.period = 1.0 / P.inner_hz
        self.create_timer(self.period, self.planning_tick, callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info(f"planning at {P.inner_hz:g} Hz, all sequences every {self.planner.outer_every} ticks")

    def parameters(self, names):
        """The values of the parameters `names`, all required (shot_planner_config.yaml): no defaults."""
        values = {name: self.declare_parameter(name, None, ParameterDescriptor(dynamic_typing=True)).value
                  for name in names}
        missing = [name for name, value in values.items() if value is None]
        if missing:
            raise RuntimeError(f"missing parameters (shot_planner_config.yaml): {', '.join(missing)}")
        return values

    ###################
    ### Callbacks   ###
    ###################
    def state_callback(self, msg):
        self.robot = body(msg)

    def episode_callback(self, msg):
        if msg.data < 0:
            self.active = False
            self.plan_publisher.publish(plan_msg(self.get_clock().now().to_msg(), self.planner.none()))
        else:
            self.planner.reset()
            self.contacts.reset()
            self.active = True
        self.get_logger().info(f"episode {msg.data}" if msg.data >= 0 else "idle")

    def puck_callback(self, msg):
        self.puck = puck = body(msg)
        robot = self.robot
        if robot is None or not self.active:
            return
        fr = self.planner.frame
        q, v = fr.pos_to_planner([puck.x, puck.y]), fr.vec_to_planner([puck.vx, puck.vy])
        if not self.contacts.update(puck.t, q, v, fr.pos_to_planner([robot.x, robot.y])):
            return
        plans = self.planner.on_contact(puck.t)
        if plans is not None:
            self.plan_publisher.publish(plan_msg(msg.header.stamp, plans))
            self.get_logger().info(f"contact, planning again in {self.P.contact_cooldown:g} s")

    def planning_tick(self):
        t_start = time.perf_counter()
        now = self.get_clock().now()
        t = 1e-9 * now.nanoseconds
        if self.t_report is None:
            self.t_report = t

        if not self.active:
            return
        plans = self.planner.tick(t, self.robot, self.puck)
        logs = self.planner.selector.logs
        self.updates += [update_json(u) for u in logs]
        logs.clear()
        if plans is not None:
            self.plan_publisher.publish(plan_msg(now.to_msg(), plans))

        if time.perf_counter() - t_start > self.period:
            self.overruns += 1
        if t - self.t_report >= REPORT_PERIOD:
            self.report(t)

    def report(self, t):
        if self.updates:
            self.get_logger().info(f"{len(self.updates)} updates, {self.overruns} ticks over "
                                   f"{1e3 * self.period:.0f} ms\n{timing_summary(self.updates)}")
        self.updates, self.overruns, self.t_report = [], 0, t

    def close(self):
        if self.nlp.pool is not None:
            self.nlp.pool.close()


def main(args=None):
    rclpy.init(args=args)
    node = ShotPlannerNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
