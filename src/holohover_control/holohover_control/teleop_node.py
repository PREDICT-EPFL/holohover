"""
ROS 2 node for velocity teleoperation of one hovercraft (namespace hN) through control_lqr: it integrates the
commanded velocity into a moving reference (a carrot) that the LQR tracks, so released keys hold position and yaw.

    cmd_vel           (in)  geometry_msgs/Twist (teleop_twist_keyboard, teleop_twist_joy): linear.x, linear.y,
                            angular.z, table frame
    state_disturbance (in)  HolohoverStateDisturbanceStamped, the EKF: the carrot starts at the first state
    state_ref         (out) HolohoverState, the carrot with the commanded velocity, for control_lqr

    ros2 run holohover_control teleop --ros-args -r __ns:=/h0
"""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from holohover_msgs.msg import HolohoverState, HolohoverStateDisturbanceStamped

CMD_TIMEOUT = 0.6  # s without cmd_vel before the velocity is zeroed: above a keyboard's auto-repeat delay


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop")
        self.period = self.declare_parameter("period", 0.01).value  # s, control_lqr's period
        # m, the carrot's bounds around the table centre: table 2.13 x 1.06 minus a margin for the hovercraft
        self.x_max = self.declare_parameter("x_max", 0.9).value
        self.y_max = self.declare_parameter("y_max", 0.4).value
        self.ref = None  # carrot (x, y, yaw)
        self.yaw = 0.0  # latest measured yaw
        self.cmd = np.zeros(3)  # latest (v_x, v_y, w_z)
        self.cmd_t = None

        self.ref_publisher = self.create_publisher(HolohoverState, "state_ref", 10)
        self.create_subscription(Twist, "cmd_vel", self.cmd_callback, 10)
        self.create_subscription(HolohoverStateDisturbanceStamped, "state_disturbance", self.state_callback, 10)
        self.create_timer(self.period, self.step)

    def cmd_callback(self, msg):
        self.cmd = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        self.cmd_t = self.get_clock().now()

    def state_callback(self, msg):
        s = msg.state_msg
        self.yaw = s.yaw
        if self.ref is None:
            self.ref = np.array([s.x, s.y, s.yaw])
            self.get_logger().info(f"carrot starts at x={s.x:.3f} y={s.y:.3f} yaw={s.yaw:.3f}")

    def step(self):
        if self.ref is None:
            return
        fresh = self.cmd_t is not None and (self.get_clock().now() - self.cmd_t).nanoseconds < CMD_TIMEOUT * 1e9
        v = self.cmd.copy() if fresh else np.zeros(3)
        self.ref += v * self.period
        lim = np.array([self.x_max, self.y_max])
        clamped = np.abs(self.ref[:2]) >= lim
        self.ref[:2] = np.clip(self.ref[:2], -lim, lim)
        v[:2][clamped] = 0.0  # a carrot held at a bound does not move
        # control_lqr does not wrap the yaw error: keep the carrot within pi of the measured yaw
        self.ref[2] = self.yaw + wrap(self.ref[2] - self.yaw)
        self.ref_publisher.publish(HolohoverState(x=self.ref[0], y=self.ref[1], v_x=v[0], v_y=v[1],
                                                  yaw=self.ref[2], w_z=v[2]))


def main():
    rclpy.init()
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
