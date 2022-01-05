import rclpy
from rclpy.node import Node, NodeNameNonExistentError
from holohover_msgs.msg import MotorControl, DroneMeasurement, Pose, DroneState, MotorControl
import numpy as np
from holohover_gnc.helpers.Holohover import Holohover
from holohover_gnc.helpers.Sensor import Sensor


class Estimator(Node):

    def __init__(self, x0, u0):
        super().__init__('Estimator')

        self.dt = 1 / 200  # Update rate

        # Subscribers
        self.camera_subscription = self.create_subscription(Pose, 'camera/robot_pose', self.updateFromCamera_callback, 10)
        self.imu_subscription = self.create_subscription(DroneMeasurement, 'drone/measurement', self.updateFromIMU_callback, 10)
        self.motor_control_subscription = self.create_subscription(MotorControl, 'drone/motor_control', self.motor_command_callback, 10)

        # Publishers
        self.state_publisher = self.create_publisher(DroneState, '/estimator/state', 10)
        self.timer = self.create_timer(self.dt, self.predict_callback)

        # Estimator state
        self.x = x0
        self.u = u0
        self.robot = Holohover(x0, u0, self.dt)

        # Kalman Filter Parameters
        # yaw, yaw_d, vx, vy, x, y
        self.Q = np.diag([1000, 1000, 1000, 1000, 1000, 1000])

        self.IMU = Sensor()  # yaw_d reading from gyro
        self.IMU.H = np.array([[0, 1, 0, 0, 0, 0]])
        self.IMU.R = np.diag([0.1])
        self.IMU.z = None
        self.IMU.K = None

        self.Camera = Sensor()  # (x, y, yaw) reading from camera
        self.Camera.H = np.array([[0, 0, 0, 0, 1, 0],
                                  [0, 0, 0, 0, 0, 1],
                                  [1, 0, 0, 0, 0, 0]])
        self.Camera.R = np.diag([1, 1, 1])
        self.Camera.z = None
        self.Camera.K = None

        self.P = self.Q

    def updateFromIMU_callback(self, msg):
        yaw_d = msg.gyro.z

        # Update Kalman Gain
        self.IMU.z = np.array([yaw_d])
        S = self.IMU.H @ self.P @ np.transpose(self.IMU.H) + self.IMU.R
        self.IMU.K = self.P @ np.transpose(self.IMU.H) @ np.linalg.inv(S)

        # Update State
        self.x = self.x + self.IMU.K @ (self.IMU.z - self.IMU.H @ self.x)

        # Update State Error Covariance
        self.P = self.P - (self.IMU.K @ self.IMU.H @ self.P)

    def updateFromCamera_callback(self, msg):
        x = msg.x
        y = msg.y
        yaw = msg.yaw

        # Update Kalman Gain
        self.Camera.z = np.array([x, y, yaw])
        S = self.Camera.H @ self.P @ np.transpose(self.Camera.H) + self.Camera.R
        self.Camera.K = self.P @ np.transpose(self.Camera.H) @ np.linalg.inv(S)

        # Update State
        self.x = self.x + self.Camera.K @ (self.Camera.z - self.Camera.H @ self.x)

        # Update State Error Covariance
        self.P = self.P - (self.Camera.K @ self.Camera.H @ self.P)

    def motor_command_callback(self, msg):
        motor_input = np.array([
            msg.motor_a_1,
            msg.motor_a_2,
            msg.motor_b_1,
            msg.motor_b_2,
            msg.motor_c_1,
            msg.motor_c_2
        ])
        thrust = self.robot.convertFromSignal(motor_input)
        self.u = self.robot.getAcceleration(thrust)

    def predict_callback(self):

        # Kalman Predict Step
        self.robot.x = self.x
        self.robot.predict(self.u)
        self.x = self.robot.x

        # Update State Error Covariance
        self.P = self.robot.A_d @ self.P @ np.transpose(self.robot.A_d) + self.Q

        # Publish
        msg = DroneState()
        msg.yaw = self.x[0]
        msg.yaw_d = self.x[1]
        msg.v_x = self.x[2]
        msg.v_y = self.x[3]
        msg.x = self.x[4]
        msg.y = self.x[5]

        self.state_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    # Initialize simulator
    x0 = np.zeros(6)
    u0 = np.zeros(3)
    estimator = Estimator(x0, u0)

    rclpy.spin(estimator)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    estimator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
