"""
Quick test script for holohover_dial nodes.

This script demonstrates how to:
1. Subscribe to control outputs
2. Publish dummy state messages
3. Verify the nodes are communicating

Run this while the nodes are running:
    python3 test_nodes.py

Then you should see:
- Path plans being received
- Control commands being generated
"""

import rclpy
from rclpy.node import Node
from holohover_msgs.msg import HolohoverStateStamped, HolohoverTrajectory, HolohoverControlStamped, HolohoverState
import numpy as np
from std_msgs.msg import Header


class TestNode(Node):
    """Test node for holohover_dial."""
    
    def __init__(self):
        super().__init__('holohover_dial_test')
        
        # Publishers
        self.h1_state_pub = self.create_publisher(HolohoverStateStamped, '/h1/state', 10)
        self.h2_state_pub = self.create_publisher(HolohoverStateStamped, '/h2/state', 10)
        self.puck_state_pub = self.create_publisher(HolohoverStateStamped, '/puck/pose', 10)
        
        # Subscribers
        self.h1_path_sub = self.create_subscription(
            HolohoverTrajectory,
            '/h1/path_plan',
            self._h1_path_callback,
            10
        )
        
        self.h1_control_sub = self.create_subscription(
            HolohoverControlStamped,
            '/h1/control',
            self._h1_control_callback,
            10
        )


        self.h2_path_sub = self.create_subscription(
            HolohoverTrajectory,
            '/h2/path_plan',
            self._h2_path_callback,
            10
        )
        
        self.h2_control_sub = self.create_subscription(
            HolohoverControlStamped,
            '/h2/control',
            self._h2_control_callback,
            10
        )
        
        # Timer for publishing state updates
        self.timer = self.create_timer(5, self._publish_state)
        self.time_counter = 0
        
        self.get_logger().info("Test node started - publishing dummy state messages")
    
    def _publish_state(self):
        """Publish a dummy state message that moves in a circle."""
        self.time_counter += 0.1
        
        # Create a simple circular motion
        radius = 0.3
        omega = 0.5  # rad/s
        angle = omega * self.time_counter
        
        h1_state = HolohoverState()
        h1_state.x = radius * np.cos(angle)
        h1_state.y = radius * np.sin(angle)
        h1_state.v_x = -radius * omega * np.sin(angle)
        h1_state.v_y = radius * omega * np.cos(angle)
        h1_state.yaw = angle
        h1_state.w_z = omega

        h1_msg = HolohoverStateStamped()
        h1_msg.header = Header()
        h1_msg.header.stamp = self.get_clock().now().to_msg()
        h1_msg.header.frame_id = "world"
        h1_msg.state_msg = h1_state

        h2_state = HolohoverState()
        h2_state.x = radius * np.cos(angle)
        h2_state.y = radius * np.sin(angle)
        h2_state.v_x = -radius * omega * np.sin(-angle)
        h2_state.v_y = radius * omega * np.cos(-angle)
        h2_state.yaw = -angle
        h2_state.w_z = -omega

        h2_msg = HolohoverStateStamped()
        h2_msg.header = Header()
        h2_msg.header.stamp = self.get_clock().now().to_msg()
        h2_msg.header.frame_id = "world"
        h2_msg.state_msg = h2_state

        puck_state = HolohoverState()
        puck_state.x = 0.0
        puck_state.y = 1.0
        puck_state.v_x = 0.0
        puck_state.v_y = 0.0
        puck_state.yaw = 0.0
        puck_state.w_z = 0.0

        puck_msg = HolohoverStateStamped()
        puck_msg.header = Header()
        puck_msg.header.stamp = self.get_clock().now().to_msg()
        puck_msg.header.frame_id = "world"
        puck_msg.state_msg = puck_state

        
        self.h1_state_pub.publish(h1_msg)
        self.h2_state_pub.publish(h2_msg)
        self.puck_state_pub.publish(puck_msg)

        self.get_logger().debug(f"Published state: h1 ({h1_state.x:.2f}, {h1_state.y:.2f}), "
            f"h2 ({h2_state.x:.2f}, {h2_state.y:.2f}), "
            f"puck ({puck_state.x:.2f}, {puck_state.y:.2f})"
        )

    def _h1_path_callback(self, msg: HolohoverTrajectory):
        """Callback when path plan is received."""
        if len(msg.state_trajectory) > 0:
            first = msg.state_trajectory[0]
            self.get_logger().info(
                f"Path received from Robot 1: {len(msg.state_trajectory)} points, "
                f"first: ({first.x:.3f}, {first.y:.3f})"
            )
    
    def _h1_control_callback(self, msg: HolohoverControlStamped):
        """Callback when control command is received."""
        self.get_logger().info(
            f"Control received from Robot 1: motors "
            f"A1={msg.motor_a_1:.3f} "
            f"B1={msg.motor_b_1:.3f} "
            f"C1={msg.motor_c_1:.3f}"
        )

    def _h2_path_callback(self, msg: HolohoverTrajectory):
        """Callback when path plan is received."""
        if len(msg.state_trajectory) > 0:
            first = msg.state_trajectory[0]
            self.get_logger().info(
                f"Path received from Robot 2: {len(msg.state_trajectory)} points, "
                f"first: ({first.x:.3f}, {first.y:.3f})"
            )

    def _h2_control_callback(self, msg: HolohoverControlStamped):
        """Callback when control command is received."""
        self.get_logger().info(
            f"Control received from Robot 2: motors "
            f"A2={msg.motor_a_2:.3f} "
            f"B2={msg.motor_b_2:.3f} "
            f"C2={msg.motor_c_2:.3f}"
        )


def main():
    rclpy.init()
    test_node = TestNode()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
