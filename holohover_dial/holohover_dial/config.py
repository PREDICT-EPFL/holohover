"""
Configuration module for holohover_dial nodes.

This module handles parameter loading and defaults for both the path planner
and control nodes. Uses ROS2 parameter system for easy runtime configuration.
"""

from dataclasses import dataclass
from typing import Optional
import rclpy
from rclpy.node import Node


@dataclass
class PathPlannerConfig:
    """Configuration for the path planner node."""

    # Planning parameters
    planning_horizon: float = 2.0  # seconds - how far ahead to plan
    goal_x: float = 0.0  # Target x position
    goal_y: float = 0.0  # Target y position
    home_x: float = 0.0  # Home/defend position x
    home_y: float = 0.0  # Home/defend position y

    # Trajectory parameters
    max_velocity: float = 2.0  # m/s - maximum velocity constraint
    update_rate: float = 30.0  # Hz - how often to replan

    @classmethod
    def from_ros_node(cls, node: Node) -> "PathPlannerConfig":
        """
        Load configuration from ROS2 parameters.

        Parameters are declared with defaults on first read.
        """
        return cls(
            planning_horizon=node.declare_parameter(
                "path_planner.planning_horizon", 2.0
            ).value,
            goal_x=node.declare_parameter("path_planner.goal_x", 0.0).value,
            goal_y=node.declare_parameter("path_planner.goal_y", 0.0).value,
            home_x=node.declare_parameter("path_planner.home_x", 0.0).value,
            home_y=node.declare_parameter("path_planner.home_y", 0.0).value,
            max_velocity=node.declare_parameter("path_planner.max_velocity", 2.0).value,
            update_rate=node.declare_parameter("path_planner.update_rate", 30.0).value,
        )


@dataclass
class ControlConfig:
    """Configuration for the control node."""

    # Control gains/weights
    position_weight: float = 1.0  # Weight for trajectory tracking
    velocity_weight: float = 0.5  # Weight for velocity smoothness
    control_limit: float = 1.0  # Maximum acceleration magnitude

    # Timing
    update_rate: float = 50.0  # Hz - control update frequency

    # Controller parameters
    kp: float = 1.0  # Proportional gain for position error
    kd: float = 0.5  # Derivative gain for velocity

    @classmethod
    def from_ros_node(cls, node: Node) -> "ControlConfig":
        """
        Load configuration from ROS2 parameters.

        Parameters are declared with defaults on first read.
        """
        return cls(
            position_weight=node.declare_parameter(
                "control.position_weight", 1.0
            ).value,
            velocity_weight=node.declare_parameter(
                "control.velocity_weight", 0.5
            ).value,
            control_limit=node.declare_parameter("control.control_limit", 1.0).value,
            update_rate=node.declare_parameter("control.update_rate", 50.0).value,
            kp=node.declare_parameter("control.kp", 1.0).value,
            kd=node.declare_parameter("control.kd", 0.5).value,
        )
