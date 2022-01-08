import rclpy
from rclpy.node import Node
from holohover_msgs.msg import MotorControl, DroneState
from geometry_msgs.msg import Twist
import numpy as np
from holohover_gnc.helpers.Holohover import Holohover
from holohover_gnc.helpers.PID import PID


class Controller(Node):

    def __init__(self):
        super().__init__('Controller')

        self.dt = 1/200  # Update rate

        # Subscriber
        self.state_subscription = self.create_subscription(DroneState, 'estimator/state', self.control_callback, 10)

        self.cmd_subscription = self.create_subscription(Twist, 'cmd_vel', self.reference_callback, 10)

        # Publisher
        self.motor_control_publisher = self.create_publisher(MotorControl, '/drone/motor_control', 10)

        # Controllers
        self.PID_vx = PID(Kp=1, Kd=0, Ki=0, ref=0, limit=100)
        self.PID_vy = PID(Kp=1, Kd=0, Ki=0, ref=0, limit=100)
        self.PID_yaw_d = PID(Kp=1, Kd=0, Ki=0, ref=0, limit=100)
        self.u = np.zeros(3)
        self.x = np.zeros(6)
        self.signal = np.empty((6, 1))
        self.thrust = np.empty((6, 1))

        self.robot = Holohover(self.x, self.u, self.dt)

    def control_callback(self, msg):
        v_x_0 = msg.v_x
        v_y_0 = msg.v_y
        yaw_d_0 = msg.yaw_d

        self.x = np.empty(6)
        self.x[0] = msg.yaw
        self.x[1] = msg.yaw_d
        self.x[2] = msg.v_x
        self.x[3] = msg.v_y
        self.x[4] = msg.x
        self.x[5] = msg.y

        acc_x = self.PID_vx.computeOutput(v_x_0)
        acc_y = self.PID_vy.computeOutput(v_y_0)
        yaw_dd = self.PID_yaw_d.computeOutput(yaw_d_0)
        self.u = np.array([acc_x, acc_y, yaw_dd])
        # self.get_logger().info('Controller Output: {}'.format(self.u))

        self.robot.x = self.x
        self.robot.u = self.u
        self.thrust = self.robot.getThrust()
        self.signal = self.robot.convertToSignal(self.thrust)

        # Cap
        # self.signal[self.signal > 1] = 1

        msg = MotorControl()
        msg.motor_a_1 = float(self.signal[0])
        msg.motor_a_2 = float(self.signal[1])
        msg.motor_b_1 = float(self.signal[2])
        msg.motor_b_2 = float(self.signal[3])
        msg.motor_c_1 = float(self.signal[4])
        msg.motor_c_2 = float(self.signal[5])

        self.motor_control_publisher.publish(msg)

    def reference_callback(self, msg):
        self.PID_vx.ref = msg.linear.x
        self.PID_vy.ref = msg.linear.y
        self.PID_yaw_d.ref = msg.linear.z


def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
