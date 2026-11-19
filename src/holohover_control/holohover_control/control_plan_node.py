"""
ROS 2 node of the plan controller for one hovercraft (namespace hN). The control itself is
plan_controller.PlanController, which python-sim runs too; this node feeds it, turns its accelerations
into motor signals (actuation.Actuation, control_lqr's actuation with holohover_common's model) and
publishes them.

    state_disturbance (in)  HolohoverStateDisturbanceStamped, the hovercraft's EKF (holohover_navigation)
                            with its disturbance estimate, table frame
    plan              (in)  HolohoverPlan of a planner (holohover_planner): its chosen candidate is
                            followed, chosen = -1 means no plan
    episode           (in)  std_msgs/Int32 of an evaluation (holohover_utils air_hockey_evaluate.py): -1
                            idles the controller (no acceleration, no disturbance compensation), an episode
                            index restarts it in defence (parameter start_idle: idle at start)
    control           (out) HolohoverControlStamped, the motor signals
    acc_ref           (out) AccelStamped, the commanded acceleration (a_x, a_y, w_dot_z), table frame
                            (not published while idle)

A timer at 1 / period runs the controller's steps from the latest state.
"""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import AccelStamped
from holohover_msgs.msg import HolohoverControlStamped, HolohoverPlan, HolohoverStateDisturbanceStamped
from std_msgs.msg import Int32

from .actuation import Actuation
from .plan import Plan
from .plan_controller import PD, PlanController
from .tracker import Tracker


# The node's parameters, all required: their values are only in control_plan_config.yaml, besides
# holohover_props_file (the hovercraft, from the launch file). start_idle is the launch file's (default false).
PARAMETERS = ("period", "plan_expiry", "acc_max", "tracker.H", "tracker.friction", "tracker.yaw_ref", "tracker.Q",
              "tracker.Q_f", "tracker.R", "defence.home", "defence.kp", "defence.kd", "holohover_props_file")


def stamp_s(stamp):
    return stamp.sec + 1e-9 * stamp.nanosec


def chosen_plan(msg):
    """The chosen candidate of a HolohoverPlan as a Plan, or None."""
    if msg.chosen < 0:
        return None
    c = msg.candidates[msg.chosen]
    return Plan(label=c.label, t0=stamp_s(c.t0), tf=c.tf,
                X=np.array([[s.x, s.y, s.v_x, s.v_y] for s in c.states], dtype=float),
                U=np.column_stack([np.asarray(c.acc_x, dtype=float), np.asarray(c.acc_y, dtype=float)]))


class ControlPlanNode(Node):
    def __init__(self):
        super().__init__("control_plan")
        p = self.parameters(PARAMETERS)
        self.period = p["period"]
        tracker = Tracker(gamma=p["tracker.friction"], acc_max=p["acc_max"], H=p["tracker.H"],
                          theta_ref=p["tracker.yaw_ref"], Q=p["tracker.Q"], Q_f=p["tracker.Q_f"], R=p["tracker.R"],
                          period=self.period)
        defence = PD(p["defence.home"], p["defence.kp"], p["defence.kd"])
        self.controller = PlanController(tracker, defence, p["acc_max"], p["plan_expiry"], log=self.get_logger().info)
        self.actuation = Actuation(p["holohover_props_file"], self.period)
        self.state = None  # latest (x, y, yaw, v_x, v_y, w_z), the controller's order
        self.state_t = None  # its stamp (s)
        self.model_state = None  # the same as (x, y, v_x, v_y, yaw, w_z), the model's order
        self.disturbance = np.zeros(3)  # latest (dist_x, dist_y, dist_yaw)
        self.active = not self.declare_parameter("start_idle", False).value  # idle: no acceleration (episode -1)

        self.control_publisher = self.create_publisher(HolohoverControlStamped, "control", qos_profile_sensor_data)
        self.acc_publisher = self.create_publisher(AccelStamped, "acc_ref", 10)
        self.create_subscription(HolohoverStateDisturbanceStamped, "state_disturbance", self.state_callback, 10)
        self.create_subscription(HolohoverPlan, "plan", self.plan_callback, 10)
        self.create_subscription(Int32, "episode", self.episode_callback, 10)
        self.create_timer(self.period, self.control_step)

    def parameters(self, names):
        """The values of the parameters `names`, all required: no defaults."""
        values = {name: self.declare_parameter(name, None, ParameterDescriptor(dynamic_typing=True)).value
                  for name in names}
        missing = [name for name, value in values.items() if value is None]
        if missing:
            raise RuntimeError(f"missing parameters (control_plan_config.yaml, holohover_props_file from the launch): "
                               f"{', '.join(missing)}")
        return values

    def now_s(self):
        return 1e-9 * self.get_clock().now().nanoseconds

    def state_callback(self, msg):
        s = msg.state_msg
        self.state = np.array([s.x, s.y, s.yaw, s.v_x, s.v_y, s.w_z], dtype=float)
        self.state_t = stamp_s(msg.header.stamp)
        self.model_state = np.array([s.x, s.y, s.v_x, s.v_y, s.yaw, s.w_z], dtype=float)
        self.disturbance = np.array([s.dist_x, s.dist_y, s.dist_yaw], dtype=float)

    def episode_callback(self, msg):
        if msg.data >= 0:
            self.controller.reset()
        self.active = msg.data >= 0

    def plan_callback(self, msg):
        self.controller.on_plan(chosen_plan(msg), self.now_s())

    def control_step(self):
        if self.state is None:
            return
        now = self.get_clock().now()
        acc = self.controller.step(1e-9 * now.nanoseconds, self.state, self.state_t) if self.active else None
        signal = self.actuation.step(self.model_state, self.disturbance, acc)

        msg = HolohoverControlStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "body"
        (msg.motor_a_1, msg.motor_a_2, msg.motor_b_1, msg.motor_b_2,
         msg.motor_c_1, msg.motor_c_2) = (float(v) for v in signal)
        self.control_publisher.publish(msg)
        if acc is not None:
            ref = AccelStamped()
            ref.header.stamp = msg.header.stamp
            ref.header.frame_id = "table"
            ref.accel.linear.x, ref.accel.linear.y, ref.accel.angular.z = (float(a) for a in acc)
            self.acc_publisher.publish(ref)


def main(args=None):
    rclpy.init(args=args)
    node = ControlPlanNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
