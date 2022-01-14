import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from holohover_msgs.msg import MotorControl, DroneMeasurement, Pose, DroneState, MotorControl
import numpy as np
from holohover_gnc.helpers.Holohover import Holohover
from holohover_gnc.helpers.Sensor import Sensor
import math 

class Estimator(Node):

    def __init__(self, x0, u0, noisy_model=False):
        super().__init__('Estimator')

        self.dt = 1 / 100  # Update rate

        # Subscribers
        self.camera_subscription = self.create_subscription(Pose, 'camera/robot_pose', self.updateFromCamera_callback, 10)
        self.imu_subscription = self.create_subscription(DroneMeasurement, 'drone/measurement', self.updateFromIMU_callback, QoSPresetProfiles.SENSOR_DATA.value)
        self.motor_control_subscription = self.create_subscription(MotorControl, 'drone/motor_control', self.motor_command_callback, QoSPresetProfiles.SENSOR_DATA.value)

        # Publishers
        self.state_publisher = self.create_publisher(DroneState, '/estimator/state', 10)
        self.timer = self.create_timer(self.dt, self.predict_callback)

        # Estimator state
        self.x = x0
        self.u = u0
        self.robot = Holohover(x0, u0, self.dt)

        if noisy_model:
            # Modify the dynamics (Noisy Model)
            self.robot.A_d[0, 0] = self.robot.A_d[0, 0] + 4
            self.robot.A_d[1, 1] = self.robot.A_d[1, 1] + 0.2
            self.robot.A_d[2, 2] = self.robot.A_d[2, 2] - 0.4
            self.robot.A_d[3, 3] = self.robot.A_d[3, 3] - 8
            self.robot.A_d[4, 2] = self.robot.A_d[4, 2] + 0.45
            self.robot.A_d[5, 3] = self.robot.A_d[5, 3] + 0.65

        # Kalman Filter Parameters
        # yaw, yaw_d, vx, vy, x, y
        self.Q = np.diag([1, 1, 1, 1, 1, 1])

        self.IMU = Sensor()  # yaw from the attitude and yaw_d reading from gyro
        self.IMU.H = np.array([[1, 0, 0, 0, 0, 0],
                               [0, 1, 0, 0, 0, 0]])
        self.IMU.R = np.diag([10000, 10])
        self.IMU.z = None
        self.IMU.K = None

        self.Camera = Sensor()  # (x, y, yaw) reading from camera
        self.Camera.H = np.array([[0, 0, 0, 0, 1, 0],
                                  [0, 0, 0, 0, 0, 1],
                                  [1, 0, 0, 0, 0, 0]])
        self.Camera.R = np.diag([100, 100, 100])
        self.Camera.z = None
        self.Camera.K = None

        self.P = self.Q

        self.Camera_YAW_OFFSET = None
        self.IMU_YAW_OFFSET = None

    def updateFromIMU_callback(self, msg):
        yaw = msg.atti.yaw
        yaw_d = msg.gyro.z

        # Offset Compensation
        if self.IMU_YAW_OFFSET is None:
            self.IMU_YAW_OFFSET = yaw
            print('The IMU OFFSET is {}'.format(self.IMU_YAW_OFFSET))

        # Correction
        yaw = yaw - self.IMU_YAW_OFFSET
        # Map yaw between 0 and 2pi
        if yaw < 0:
            yaw += math.ceil(-yaw / (2 * math.pi)) * 2 * math.pi
        elif yaw > 2 * math.pi:
            yaw -= math.floor(yaw / (2 * math.pi)) * 2 * math.pi
        #print('IMU yaw is {}'.format(yaw))

        # Update Kalman Gain
        self.IMU.z = np.array([yaw, yaw_d])
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


        # Offset Compensation
        if self.Camera_YAW_OFFSET is None:
            self.Camera_YAW_OFFSET = yaw
            print('The CAMERA OFFSET is {}'.format(self.Camera_YAW_OFFSET))

        # Correction
        yaw = yaw - self.Camera_YAW_OFFSET
        # Map yaw between 0 and 2pi
        # if yaw < 0:
        #     yaw += math.ceil(-yaw / (2 * math.pi)) * 2 * math.pi
        # elif yaw > 2 * math.pi:
        #     yaw -= math.floor(yaw / (2 * math.pi)) * 2 * math.pi

        print('Camera yaw is {}'.format(yaw))

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
    estimator = Estimator(x0, u0, noisy_model=False)

    rclpy.spin(estimator)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    estimator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
