"""
Control Node for holohover_dial

This node takes a desired trajectory (path plan) and the current state, then
computes motor commands to track that trajectory. It implements a simple
trajectory tracking controller.

Subscriptions:
    - /holohover/state: Current state
    - /holohover/path_plan: Desired trajectory from path planner

Publications:
    - /holohover/control: Motor commands for the holohover
"""

import numpy as np
from typing import Optional
import rclpy
from rclpy.node import Node

# Message types
from holohover_msgs.msg import HolohoverStateStamped, HolohoverTrajectory, HolohoverControlStamped

# Local imports
from holohover_dial.config import ControlConfig


class ControlNode(Node):
    """
    Computes control inputs to track a desired trajectory.
    
    This node implements a trajectory tracking controller. It uses:
    - PD control for position tracking
    - Velocity feedforward for smooth motion
    
    Subscriptions:
        - /holohover/state: Current state of the holohover
        - /holohover/path_plan: Desired trajectory to follow
    
    Publications:
        - /holohover/control: Motor acceleration commands
    """

    def __init__(self):
        super().__init__('control_node')
        
        # Load configuration from ROS2 parameters
        self.config = ControlConfig.from_ros_node(self)
        self.get_logger().info(f"Control Node initialized with config: {self.config}")
        
        # State tracking
        self.current_state: Optional[HolohoverStateStamped] = None
        self.current_trajectory: Optional[HolohoverTrajectory] = None
        self.current_trajectory_idx: int = 0  # Which point in trajectory we're targeting
        
        # Create subscriptions and publishers
        self._init_subscriptions()
        self._init_publishers()
        
        # Create a timer for periodic control updates
        period = 1.0 / self.config.update_rate  # Convert Hz to seconds
        self.control_timer = self.create_timer(3, self._control_callback)
        self.get_logger().info(f"Control timer set to {self.config.update_rate} Hz")

    def _init_subscriptions(self):
        """Initialize ROS2 subscriptions."""
        # Subscribe to current state
        self.state_subscription = self.create_subscription(
            HolohoverStateStamped,
            'state',
            self._state_callback,
            10
        )
        self.get_logger().info("Subscribed to state")
        
        # Subscribe to desired trajectory
        self.trajectory_subscription = self.create_subscription(
            HolohoverTrajectory,
            'path_plan',
            self._trajectory_callback,
            10
        )
        self.get_logger().info("Subscribed to path_plan")

    def _init_publishers(self):
        """Initialize ROS2 publishers."""
        # Publish control commands
        self.control_publisher = self.create_publisher(
            HolohoverControlStamped,
            'control',
            10
        )
        self.get_logger().info("Publishing to control")

    def _state_callback(self, msg: HolohoverStateStamped):
        """
        Callback when new state is received.
        
        Args:
            msg: The current state of the holohover
        """
        self.current_state = msg

    def _trajectory_callback(self, msg: HolohoverTrajectory):
        """
        Callback when new trajectory is received.
        
        Args:
            msg: The desired trajectory from the path planner
        """
        self.current_trajectory = msg
        self.current_trajectory_idx = 0  # Reset to first point when new trajectory arrives

    def _control_callback(self):
        """
        Timer callback that computes and publishes control commands.
        
        This is called periodically and computes the motor accelerations
        needed to track the current trajectory.
        """
        # Only proceed if we have both state and trajectory
        if self.current_state is None or self.current_trajectory is None:
            self.get_logger().warn("Waiting for state and/or trajectory")
            return
        
        if len(self.current_trajectory.state_trajectory) == 0:
            self.get_logger().warn("Trajectory is empty")
            return
        
        # Compute control commands
        control = self._compute_control()
        
        # Publish the control commands
        if control is not None:
            self._publish_control(control)
        
        # Advance to next trajectory point for next control step
        self._advance_trajectory_index()

    def _compute_control(self) -> Optional[np.ndarray]:
        """
        Compute control acceleration commands using trajectory tracking controller.
        
        This implements a simple PD controller:
        - P term: corrects position error
        - D term: corrects velocity error
        - Feedforward: adds desired velocity from trajectory
        
        Returns:
            2D numpy array [ax, ay] of acceleration commands, or None if not ready
        """
        # Get current state
        current_pos = np.array([self.current_state.state_msg.x, self.current_state.state_msg.y])
        current_vel = np.array([self.current_state.state_msg.v_x, self.current_state.state_msg.v_y])
        
        # Get desired state from current trajectory point
        desired_state = self.current_trajectory.state_trajectory[self.current_trajectory_idx]
        desired_pos = np.array([desired_state.x, desired_state.y])
        desired_vel = np.array([desired_state.v_x, desired_state.v_y])
        
        # Compute errors
        pos_error = desired_pos - current_pos  # How far off we are from desired position
        vel_error = desired_vel - current_vel  # How far off we are from desired velocity
        
        # PD controller with feedforward
        # u = kp * pos_error + kd * vel_error + desired_accel
        u_feedback = self.config.kp * pos_error + self.config.kd * vel_error
        
        # Feedforward term: we want to go toward goal at desired velocity
        # In a more complete controller, you might compute desired acceleration from trajectory
        u_feedforward = desired_vel  # Simple feedforward: try to achieve desired velocity
        
        # Combine feedback and feedforward
        acceleration = u_feedback + 0.5 * u_feedforward  # Weight feedforward by 0.5
        
        # Limit the acceleration magnitude to control_limit
        accel_mag = np.linalg.norm(acceleration)
        if accel_mag > self.config.control_limit:
            acceleration = acceleration / accel_mag * self.config.control_limit
        
        return acceleration

    def _advance_trajectory_index(self):
        """
        Advance to the next point in the trajectory.
        
        This allows the controller to "walk" through the trajectory points
        as time progresses. When reaching the end, it stays on the last point.
        """
        if self.current_trajectory is None:
            return
        
        max_idx = len(self.current_trajectory.state_trajectory) - 1
        if self.current_trajectory_idx < max_idx:
            self.current_trajectory_idx += 1

    def _publish_control(self, acceleration: np.ndarray):
        """
        Publish the computed control commands as motor accelerations.
        
        Note: The actual motor commands depend on the holohover's kinematics.
        This is a simplified version that publishes the desired acceleration.
        You'll need to convert this to motor commands based on your hardware setup.
        
        Args:
            acceleration: 2D array [ax, ay] of accelerations
        """
        control_msg = HolohoverControlStamped()
        control_msg.header.stamp = self.get_clock().now().to_msg()
        control_msg.header.frame_id = "holohover"
        
        # For now, we'll send the acceleration directly to all motors
        # In practice, you'll need to map from [ax, ay] to individual motor commands
        # based on your holohover's three-motor configuration.
        
        # Simple mapping: distribute acceleration equally to all motors
        # This is a placeholder - modify based on your actual motor configuration!
        motor_commands = self._acceleration_to_motor_commands(acceleration)
        
        control_msg.motor_a_1 = motor_commands[0]
        control_msg.motor_a_2 = motor_commands[0]
        control_msg.motor_b_1 = motor_commands[1]
        control_msg.motor_b_2 = motor_commands[1]
        control_msg.motor_c_1 = motor_commands[2]
        control_msg.motor_c_2 = motor_commands[2]
        
        self.control_publisher.publish(control_msg)

    def _acceleration_to_motor_commands(self, acceleration: np.ndarray) -> np.ndarray:
        """
        Convert desired acceleration [ax, ay] to motor commands.
        
        This is a placeholder implementation. You need to implement the
        inverse kinematics for your holohover setup.
        
        For a 3-motor holohover configuration (typically at 120° angles),
        you would use something like:
            motor_1 = k1 * ax + k2 * ay
            motor_2 = k3 * ax + k4 * ay
            motor_3 = k5 * ax + k6 * ay
        
        Where k1-k6 are computed from the motor geometry.
        
        Args:
            acceleration: 2D desired acceleration [ax, ay]
        
        Returns:
            3D array of motor commands
        """
        # Placeholder: simple proportional scaling
        # TODO: Replace with actual kinematics for your platform
        motor_ax = acceleration[0]
        motor_ay = acceleration[1]
        
        # Three motors at 0°, 120°, 240° configuration (example)
        # Motor command = ax * cos(angle) + ay * sin(angle)
        motor_1 = motor_ax * 1.0 + motor_ay * 0.0  # Motor at 0°
        motor_2 = motor_ax * np.cos(2*np.pi/3) + motor_ay * np.sin(2*np.pi/3)  # 120°
        motor_3 = motor_ax * np.cos(4*np.pi/3) + motor_ay * np.sin(4*np.pi/3)  # 240°
        
        return np.array([motor_1, motor_2, motor_3])


def main(args=None):
    """Entry point for control node."""
    rclpy.init(args=args)
    node = ControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
