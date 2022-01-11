import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from holohover_msgs.msg import DroneState, DroneMeasurement, Pose, MotorControl
import numpy as np
from holohover_gnc.helpers.Holohover import Holohover


class Simulator(Node):

    def __init__(self, x0, u0):
        super().__init__('Simulator')

        self.state_publisher = self.create_publisher(DroneState, '/simulator/state', 10)
        state_timer_period = 1 / 200  # 200Hz
        self.state_timer = self.create_timer(state_timer_period, self.state_callback)

        self.imu_publisher = self.create_publisher(DroneMeasurement, '/drone/measurement', QoSPresetProfiles.SENSOR_DATA.value)
        imu_timer_period = 1 / 200  # 200Hz
        self.imu_timer = self.create_timer(imu_timer_period, self.measurement_callback)

        self.camera_publisher = self.create_publisher(Pose, '/camera/robot_pose', 10)
        camera_period = 1 / 30  # 30Hz
        self.camera_timer = self.create_timer(camera_period, self.pose_callback)

        self.motor_control_subscription = self.create_subscription(MotorControl, 'drone/motor_control', self.listener_callback, QoSPresetProfiles.SENSOR_DATA.value)

        self.robot = Holohover(x0, u0, state_timer_period)
        self.u = u0

    def state_callback(self):
        # simulate robot by one time step
        self.robot.predict(self.u)

        msg = DroneState()
        msg.yaw = float(self.robot.x[0])
        msg.yaw_d = float(self.robot.x[1])
        msg.v_x = float(self.robot.x[2])
        msg.v_y = float(self.robot.x[3])
        msg.x = float(self.robot.x[4])
        msg.y = float(self.robot.x[5])

        self.state_publisher.publish(msg)

    def measurement_callback(self):
        msg = DroneMeasurement()

        # Attitude
        msg.atti.roll = float(0)
        msg.atti.pitch = float(0)
        msg.atti.yaw = float(self.robot.x[0])

        # Accelerometer and Gyro
        gyro, acc = self.robot.getIMUReading(noisy=True)
        msg.gyro.x = float(gyro[0])
        msg.gyro.y = float(gyro[1])
        msg.gyro.z = float(gyro[2])

        msg.acc.x = float(acc[0])
        msg.acc.y = float(acc[1])
        msg.acc.z = float(acc[2])

        self.imu_publisher.publish(msg)

    def pose_callback(self):
        msg = Pose()

        pose = self.robot.getCameraReading(noisy=True)
        msg.x = pose[0]
        msg.y = pose[1]
        msg.yaw = pose[2]

        self.camera_publisher.publish(msg)

    def listener_callback(self, msg):
        # Update the thrust input
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


def main(args=None):
    rclpy.init(args=args)

    # Initialize simulator
    x0 = np.zeros(6)
    u0 = np.zeros(3)
    simulator = Simulator(x0, u0)

    rclpy.spin(simulator)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    simulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
