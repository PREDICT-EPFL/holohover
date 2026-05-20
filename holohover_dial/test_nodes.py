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
        self.state_pub = self.create_publisher(HolohoverStateStamped, '/holohover/state', 10)
        
        # Subscribers
        self.path_sub = self.create_subscription(
            HolohoverTrajectory,
            '/holohover/path_plan',
            self._path_callback,
            10
        )
        
        self.control_sub = self.create_subscription(
            HolohoverControlStamped,
            '/holohover/control',
            self._control_callback,
            10
        )
        
        # Timer for publishing state updates
        self.timer = self.create_timer(0.1, self._publish_state)
        self.time_counter = 0
        
        self.get_logger().info("Test node started - publishing dummy state messages")
    
    def _publish_state(self):
        """Publish a dummy state message that moves in a circle."""
        self.time_counter += 0.1
        
        # Create a simple circular motion
        radius = 0.3
        omega = 0.5  # rad/s
        angle = omega * self.time_counter
        
        state = HolohoverState()
        state.x = radius * np.cos(angle)
        state.y = radius * np.sin(angle)
        state.v_x = -radius * omega * np.sin(angle)
        state.v_y = radius * omega * np.cos(angle)
        state.yaw = angle
        state.w_z = omega
        
        msg = HolohoverStateStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.state_msg = state
        
        self.state_pub.publish(msg)
    
    def _path_callback(self, msg: HolohoverTrajectory):
        """Callback when path plan is received."""
        if len(msg.state_trajectory) > 0:
            first = msg.state_trajectory[0]
            self.get_logger().info(
                f"Path received: {len(msg.state_trajectory)} points, "
                f"first: ({first.x:.3f}, {first.y:.3f})"
            )
    
    def _control_callback(self, msg: HolohoverControlStamped):
        """Callback when control command is received."""
        self.get_logger().info(
            f"Control received: motors "
            f"A1={msg.motor_a_1:.3f} "
            f"B1={msg.motor_b_1:.3f} "
            f"C1={msg.motor_c_1:.3f}"
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
