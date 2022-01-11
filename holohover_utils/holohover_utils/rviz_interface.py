import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.duration import Duration
from holohover_msgs.msg import DroneState, MotorControl
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point
from holohover_utils.tf_transformations import quaternion_from_euler
from holohover_gnc.helpers.Holohover import Holohover


class RVizInterface(Node):

    def __init__(self):
        super().__init__('RVizInterface')

        self.x = np.zeros(6)
        self.robot = Holohover(self.x, np.zeros(3))
        self.thrusts = np.zeros(6)

        # Subscriber
        self.state_subscription = self.create_subscription(DroneState, '/estimator/state', self.state_callback, 10)
        self.motor_control_subscription = self.create_subscription(MotorControl, '/drone/motor_control', self.control_callback, QoSPresetProfiles.SENSOR_DATA.value)

        # Publisher
        self.viz_publisher = self.create_publisher(MarkerArray, '/holohover_visualization', 10)

        # Timer
        self.timer = self.create_timer(1 / 25, self.publish_callback)  # publish at 25 Hz

        # Init markers
        self.id_counter = 0

        self.holohover_marker = self.create_marker('holohover', 0.75, 0.75, 0.75)
        self.holohover_marker.type = Marker.MESH_RESOURCE
        self.holohover_marker.mesh_resource = 'package://holohover_utils/gui/holohover.stl'
        self.holohover_marker.scale.x = 1.0
        self.holohover_marker.scale.y = 1.0
        self.holohover_marker.scale.z = 1.0

        self.thrust_vectors = []
        for i in range(6):
            thrust_vector = self.create_marker(f'thrust vector {i}', 1.0, 0.5, 0.0)
            thrust_vector.type = Marker.ARROW
            thrust_vector.scale.x = 0.01
            thrust_vector.scale.y = 0.02
            thrust_vector.scale.z = 0.02
            self.thrust_vectors.append(thrust_vector)

    def create_marker(self, ns, r, g, b, a=1.0, frame_id='world'):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.action = Marker.ADD
        marker.ns = ns
        marker.id = self.id_counter
        self.id_counter += 1
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a

        return marker

    def state_callback(self, msg):
        self.x = np.array([
            msg.yaw,
            msg.yaw_d,
            msg.v_x,
            msg.v_y,
            msg.x,
            msg.y,
        ])
        self.robot.x = self.x

    def control_callback(self, msg):
        self.thrusts = np.array([
            msg.motor_a_1,
            msg.motor_a_2,
            msg.motor_b_1,
            msg.motor_b_2,
            msg.motor_c_1,
            msg.motor_c_2,
        ])

    def publish_callback(self):
        # Holohover Model
        self.holohover_marker.header.stamp = self.get_clock().now().to_msg()
        self.holohover_marker.lifetime = Duration().to_msg()
        pose = Pose()
        pose.position.x = self.x[4]
        pose.position.y = self.x[5]
        pose.position.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, self.x[0])
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        self.holohover_marker.pose = pose

        # Thrust Arrows
        T = self.robot._Holohover__getBodytoWorld()
        radial_vectors = self.robot._Holohover__getRadialVector()
        # We are taking the inverted direction vectors to visualize the air flow
        direction_vectors = -self.robot._Holohover__getDirectionVector()

        propeller_height = 0.045
        thrust_scaling = 0.2

        for i in range(6):
            start_vec = T @ radial_vectors[i, :]
            end_vec = T @ (radial_vectors[i, :] + direction_vectors[i, :] * self.thrusts[i] * thrust_scaling)
            
            start_vec[0] += self.x[4]
            start_vec[1] += self.x[5]
            start_vec[2] += propeller_height
            end_vec[0] += self.x[4]
            end_vec[1] += self.x[5]
            end_vec[2] += propeller_height

            start = Point()
            start.x = start_vec[0]
            start.y = start_vec[1]
            start.z = start_vec[2]
            
            end = Point()
            end.x = end_vec[0]
            end.y = end_vec[1]
            end.z = end_vec[2]

            self.thrust_vectors[i].header.stamp = self.get_clock().now().to_msg()
            self.thrust_vectors[i].lifetime = Duration().to_msg()
            self.thrust_vectors[i].points = [start, end]

        # Publish
        markers = [self.holohover_marker]
        for i in range(6):
            markers.append(self.thrust_vectors[i])
        self.viz_publisher.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args)

    rviz_interface = RVizInterface()

    rclpy.spin(rviz_interface)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rviz_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
