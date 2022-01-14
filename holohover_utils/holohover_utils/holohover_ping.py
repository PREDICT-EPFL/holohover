import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.time import Time
from std_msgs.msg import Header


class HolohoverPing(Node):

    def __init__(self):
        super().__init__('HolohoverPing')

        # Subscriber
        self.pong_subscription = self.create_subscription(Header, '/drone/pong', self.pong_callback, QoSPresetProfiles.SENSOR_DATA.value)

        # Publisher
        self.ping_publisher = self.create_publisher(Header, '/drone/ping', QoSPresetProfiles.SENSOR_DATA.value)

        # Timer
        self.timer = self.create_timer(2, self.publish_ping_callback)

    def publish_ping_callback(self):
        header = Header()
        header.frame_id = ''
        header.stamp = self.get_clock().now().to_msg()
        self.ping_publisher.publish(header)

    def pong_callback(self, msg):
        ping_time = Time.from_msg(msg.stamp)
        duration = self.get_clock().now() - ping_time
        self.get_logger().info(f'Received Ping in {duration.nanoseconds * 1e-6} ms')


def main(args=None):
    rclpy.init(args=args)

    holohover_ping = HolohoverPing()

    rclpy.spin(holohover_ping)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    holohover_ping.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
