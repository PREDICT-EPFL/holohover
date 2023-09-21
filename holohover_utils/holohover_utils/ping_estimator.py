#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.time import Time
from std_msgs.msg import Header


class PingEstimator(Node):

    def __init__(self):
        super().__init__('PingEstimator')

        # Subscriber
        self.pong_subscription = self.create_subscription(Header, '/drone/pong', self.pong_callback, QoSPresetProfiles.SENSOR_DATA.value)

        # Publisher
        self.ping_publisher = self.create_publisher(Header, '/drone/ping', QoSPresetProfiles.SENSOR_DATA.value)

        # Timer
        self.timer = self.create_timer(1, self.publish_ping_callback)

        self.ping = Header()
        self.ping.frame_id = ''

    def publish_ping_callback(self):
        self.ping.stamp = self.get_clock().now().to_msg()
        self.ping_publisher.publish(self.ping)

    def pong_callback(self, pong):
        ping_time = Time.from_msg(self.ping.stamp)
        pong_time = Time.from_msg(pong.stamp)
        current_time = self.get_clock().now()
        duration_agent_car = pong_time - ping_time
        duration_car_agent = current_time - pong_time
        duration_RTT = current_time - ping_time
        self.get_logger().info(f'Received Ping: Agent     -> Holohover {duration_agent_car.nanoseconds * 1e-6:.3f} ms')
        self.get_logger().info(f'               Holohover -> Agent     {duration_car_agent.nanoseconds * 1e-6:.3f} ms')
        self.get_logger().info(f'               Round-Trip-Time        {duration_RTT.nanoseconds * 1e-6:.3f} ms')


def main(args=None):
    rclpy.init(args=args)

    ping_estimator = PingEstimator()

    rclpy.spin(ping_estimator)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ping_estimator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
