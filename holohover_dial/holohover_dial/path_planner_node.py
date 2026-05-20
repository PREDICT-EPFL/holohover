"""
Path Planner Node for holohover_dial

This node takes the current state of the holohover and generates a desired
trajectory (sequence of positions) that the control node will try to follow.

The path planner is where high-level planning logic lives. For now, it implements
a simple template that tracks toward a goal position.
"""

import numpy as np
from typing import List
import rclpy
from rclpy.node import Node
from rclpy.timer import Rate

# Message types
from holohover_msgs.msg import HolohoverStateStamped, HolohoverTrajectory, HolohoverState

# Local imports
from holohover_dial.config import PathPlannerConfig


class PathPlannerNode(Node):
    """
    Plans a trajectory based on current state.
    
    Subscriptions:
        - /holohover/state: Current state of the holohover
    
    Publications:
        - /holohover/path_plan: Desired trajectory to follow
    """

    def __init__(self):
        super().__init__('path_planner')
        
        # Load configuration from ROS2 parameters
        self.config = PathPlannerConfig.from_ros_node(self)
        self.get_logger().info(f"Path Planner initialized with config: {self.config}")
        
        # State tracking
        self.current_state: HolohoverState = None
        self.path_plan: List[HolohoverState] = []
        
        # Create subscriptions and publishers
        self._init_subscriptions()
        self._init_publishers()
        
        # Create a timer for periodic planning updates
        period = 1.0 / self.config.update_rate  # Convert Hz to seconds
        self.planning_timer = self.create_timer(period, self._planning_callback)
        self.get_logger().info(f"Planning timer set to {self.config.update_rate} Hz")

    def _init_subscriptions(self):
        """Initialize ROS2 subscriptions."""
        # Subscribe to current state
        self.state_subscription = self.create_subscription(
            HolohoverStateStamped,
            '/holohover/state',
            self._state_callback,
            10  # QoS history depth
        )
        self.get_logger().info("Subscribed to /holohover/state")

    def _init_publishers(self):
        """Initialize ROS2 publishers."""
        # Publish planned trajectory
        self.path_plan_publisher = self.create_publisher(
            HolohoverTrajectory,
            '/holohover/path_plan',
            10
        )
        self.get_logger().info("Publishing to /holohover/path_plan")

    def _state_callback(self, msg: HolohoverStateStamped):
        """
        Callback when new state is received.
        
        Args:
            msg: The current state of the holohover
        """
        self.current_state = msg.state_msg

    def _planning_callback(self):
        """
        Timer callback that generates and publishes a new path plan.
        
        This is called periodically (at update_rate frequency) and should
        generate a trajectory based on the current state.
        """
        # Only proceed if we have received at least one state message
        if self.current_state is None:
            # self.get_logger().warn("No state received yet")
            return

        # Generate a trajectory (this is the main planning logic)
        trajectory = self._generate_trajectory()
        
        # Publish the trajectory
        if trajectory:
            self._publish_trajectory(trajectory)

    def _generate_trajectory(self) -> List[HolohoverState]:
        """
        Generate a desired trajectory based on current state.
        
        This is the core planning algorithm. Currently implements a simple
        linear interpolation toward the goal. Replace this with your planning
        algorithm (e.g., path planning, optimization, diffusion models, etc.)
        
        Returns:
            List of HolohoverState messages representing desired positions
        """
        trajectory = []
        
        # Get current position and velocity
        current_pos = np.array([self.current_state.x, self.current_state.y])
        current_vel = np.array([self.current_state.v_x, self.current_state.v_y])
        goal_pos = np.array([self.config.goal_x, self.config.goal_y])
        
        # Simple planning: linearly move toward goal
        # You can replace this with more sophisticated planning
        
        # Number of points in the trajectory
        n_points = int(self.config.planning_horizon * self.config.update_rate)
        n_points = max(n_points, 5)  # At least 5 points
        
        for i in range(n_points):
            # Linear interpolation parameter (0 to 1)
            alpha = (i + 1) / n_points
            
            # Interpolate position toward goal
            desired_pos = current_pos + alpha * (goal_pos - current_pos)
            
            # Compute desired velocity as simple proportional control
            # (reduce velocity as we get close to goal)
            direction = goal_pos - current_pos
            distance = np.linalg.norm(direction)
            
            if distance > 1e-3:
                # Move toward goal with velocity proportional to distance
                desired_vel = (direction / distance) * min(self.config.max_velocity, distance)
            else:
                desired_vel = np.array([0.0, 0.0])
            
            # Create HolohoverState message
            state = HolohoverState()
            state.x = float(desired_pos[0])
            state.y = float(desired_pos[1])
            state.v_x = float(desired_vel[0])
            state.v_y = float(desired_vel[1])
            state.yaw = self.current_state.yaw  # Keep current yaw (not planning orientation)
            state.w_z = 0.0  # No angular velocity in this simple planner
            
            trajectory.append(state)
        
        return trajectory

    def _publish_trajectory(self, trajectory: List[HolohoverState]):
        """
        Publish the generated trajectory.
        
        Args:
            trajectory: List of desired HolohoverState messages
        """
        # Create trajectory message
        traj_msg = HolohoverTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.header.frame_id = "world"  # Reference frame
        traj_msg.state_trajectory = trajectory
        
        # Publish
        self.path_plan_publisher.publish(traj_msg)


def main(args=None):
    """Entry point for path planner node."""
    rclpy.init(args=args)
    node = PathPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
