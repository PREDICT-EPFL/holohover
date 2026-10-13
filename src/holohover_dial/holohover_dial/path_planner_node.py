"""
Path Planner Node for holohover_dial

This node takes the current state of the holohover and generates a desired
path_plan (sequence of positions) that the control node will try to follow.

The path planner is where high-level planning logic lives. For now, it implements
a simple template that tracks toward a goal position.
"""

from typing import List
import rclpy
from rclpy.node import Node
from rclpy.timer import Rate

import jax.numpy as jnp

# Message types
from holohover_msgs.msg import HolohoverStateStamped, HolohoverTrajectory, HolohoverPathPlan, HolohoverState

# Local imports
from holohover_dial.config import PathPlannerConfig

# from holohover_dial.path_planner.verify import function  # Test import to verify JAX is working
from holohover_dial.path_planner.path_planner import build_dial_path_planner
from holohover_dial.path_planner.controller import build_lqr_tracking_controller


class PathPlannerNode(Node):
    """
    Plans a trajectory based on current state.
    
    Subscriptions:
        - /holohover/state: Current state of the holohover
    
    Publications:
        - /holohover/path_plan: Desired path plan to follow
    """

    def __init__(self):
        super().__init__('path_planner')
        

        self.config = PathPlannerConfig.from_ros_node(self)

        self.get_logger().info(f"Path Planner initialized with config: {self.config}")
        print("Testing JAX function:", function())  # Test that JAX is working
        self.current_state: HolohoverState = None
        self.path_plan: HolohoverPathPlan = None
        
        self._init_subscriptions()
        self._init_publishers()
        
        dt = 1.0 / self.config.dial.hz  # Convert Hz to seconds
        self.planning_timer = self.create_timer(dt * 100, self._planning_callback)
        self.get_logger().info(f"Planning timer set to {self.config.dial.hz} Hz")

        self.get_logger().info(f"My home is at {self.config.hovercraft.player.defensive_position_rel}")

        # self.def_controller = build_lqr_tracking_controller(np.zeros(6), robot_id=0)
        # self.imagined_enemy_controller = build_lqr_tracking_controller(np.zeros(6), robot_id=1)
        # self.planner = build_dial_path_planner(
        #     0,
        #     self.def_controller,
        #     self.imagined_enemy_controller,
        # )

    def _init_subscriptions(self):
        """Initialize ROS2 subscriptions."""
        # Subscribe to current state
        self.state_subscription = self.create_subscription(
            HolohoverStateStamped,
            'state',
            self._state_callback,
            10  # QoS history depth
        )
        self.get_logger().info("Subscribed to state")

    def _init_publishers(self):
        """Initialize ROS2 publishers."""
        # Publish planned trajectory
        self.path_plan_publisher = self.create_publisher(
            HolohoverPathPlan,
            'path_plan',
            10
        )
        self.get_logger().info("Publishing to path_plan")

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

        # Generate a path plan (this is the main planning logic)
        path_plan = self._generate_path_plan()
        
        # Publish the path plan
        if path_plan:
            self._publish_path_plan(path_plan)

    def _generate_path_plan(self) -> List[HolohoverState]:
        """
        Generate a desired path plan based on current state.
        
        This is the core planning algorithm. Currently implements a simple
        linear interpolation toward the goal. Replace this with your planning
        algorithm (e.g., path planning, optimization, diffusion models, etc.)
        
        Returns:
            List of HolohoverState messages representing desired positions
        """
        
        path_plan = HolohoverPathPlan()
        path_plan.header.stamp = self.get_clock().now().to_msg()
        return path_plan

    def _publish_path_plan(self, path_plan: List[HolohoverState]):
        """
        Publish the generated path plan.
        
        Args:
            path_plan: List of desired HolohoverState messages
        """
        # Create path plan message
        traj_msg = HolohoverPathPlan()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.header.frame_id = "world"  # Reference frame
        traj_msg.state_path_plan = path_plan
        
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
