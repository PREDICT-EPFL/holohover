"""
Configuration module for holohover_dial nodes.

This module handles parameter loading and defaults for both the path planner
and control nodes. Uses ROS2 parameter system for easy runtime configuration.
"""

from dataclasses import dataclass
from typing import Optional
import rclpy
from rclpy.node import Node


def _hovercraft_id_from_node(node: Node) -> int:
    namespace = node.get_namespace().strip("/")
    if namespace.endswith("h2"):
        return 2
    if namespace.endswith("h1"):
        return 1
    return 1


@dataclass
class HovercraftRoleConfig:
    defensive_position_rel: float = 0.0
    initial_position: list = None
    initial_velocity: list = None

    @classmethod
    def from_ros_node(cls, node: Node, id: Optional[int] = None) -> "HovercraftRoleConfig":
        if id is None:
            id = _hovercraft_id_from_node(node)
        return cls(
            defensive_position_rel=node.declare_parameter(
                f"hovercraft.h{id}.defensive_position_rel", 0.0
            ).value,
            initial_position=node.declare_parameter(
                f"hovercraft.h{id}.initial_position", [0.0, 0.0, 0.0]
            ).value,
            initial_velocity=node.declare_parameter(
                f"hovercraft.h{id}.initial_velocity", [0.0, 0.0, 0.0]
            ).value,
        )
    

@dataclass
class TableConfig:
    width: float = 2.13  # meters
    height: float = 1.06  # meters
    goal_width: float = 0.34  # meters
    friction: float = 0.0  # friction coefficient

    @classmethod
    def from_ros_node(cls, node: Node) -> "TableConfig":
        return cls(
            width=node.declare_parameter("table.width", 2.13).value,
            height=node.declare_parameter("table.height", 1.06).value,
            goal_width=node.declare_parameter("table.goal_width", 0.34).value,
            friction=node.declare_parameter("table.friction", 0.0).value,
        )
    

@dataclass
class HovercraftConfig:
    radius: float = 0.07  # meters
    mass: float = 0.116  # kg
    u_limits: list = None  # max control xy-force and torque
    player: HovercraftRoleConfig = None
    opponent: HovercraftRoleConfig = None

    @classmethod
    def from_ros_node(cls, node: Node, id: Optional[int] = None) -> "HovercraftConfig":
        if id is None:
            id = _hovercraft_id_from_node(node)
        return cls(
            radius=node.declare_parameter("hovercraft.radius", 0.07).value,
            mass=node.declare_parameter("hovercraft.mass", 0.116).value,
            u_limits=node.declare_parameter(
                "hovercraft.u_limits", [0.35, 0.35, 0.005]
            ).value,
            player=HovercraftRoleConfig.from_ros_node(node, id),
            opponent=HovercraftRoleConfig.from_ros_node(node, 1 - id),
        )
    

@dataclass
class PuckConfig:
    radius: float = 0.05  # meters
    mass: float = 0.023  # kg

    @classmethod
    def from_ros_node(cls, node: Node) -> "PuckConfig":
        return cls(
            radius=node.declare_parameter("puck.radius", 0.05).value,
            mass=node.declare_parameter("puck.mass", 0.023).value,
        )


@dataclass
class DIALConfig:
    hz: float = 10.0  # Hz - how often to run DIAL optimization
    H: int = 15  # MPC horizon length in timesteps
    Nw: int = 2000  # Number of rollouts/samples to generate
    N: int = 8  # Number of top trajectories to use for updates
    n_knots: int = 5  # Number of knot points for trajectory parameterization
    update_temperature: float = 10.0  # Temperature parameter for softmax weighting
    beta1: float = 0.25  # Momentum term for trajectory updates
    beta2: float = 1.0  # Momentum term for trajectory updates
    path_planning_scale: list = None  # Scaling for state components in cost
    tf_scale: float = 0.01  # Scaling for time component in cost
    Q: list = None  # State cost weights
    R: list = None  # Control cost weights
    goal_check_horizon_s: float = 1.0  # seconds - how far ahead to check for goal condition
    goal_check_hz: float = 25.0  # Hz - refresh rate for checking goal condition

    @classmethod
    def from_ros_node(cls, node: Node) -> "DIALConfig":
        if cls.path_planning_scale is None:
            cls.path_planning_scale = [0.01, 0.01, 0.5]
        if cls.Q is None:
            cls.Q = [10, 10, 10, 5, 5, 5]
        if cls.R is None:
            cls.R = [0.001, 0.01, 0.05]

        return cls(
            hz=node.declare_parameter("dial.hz", 10.0).value,
            H=node.declare_parameter("dial.H", 15).value,
            Nw=node.declare_parameter("dial.Nw", 2000).value,
            N=node.declare_parameter("dial.N", 8).value,
            n_knots=node.declare_parameter("dial.n_knots", 5).value,
            update_temperature=node.declare_parameter(
                "dial.update_temperature", 10.0
            ).value,
            beta1=node.declare_parameter("dial.beta1", 0.25).value,
            beta2=node.declare_parameter("dial.beta2", 1.0).value,
            path_planning_scale=node.declare_parameter(
                "dial.path_planning_scale", [0.01, 0.01, 0.5]
            ).value,
            tf_scale=node.declare_parameter("dial.tf_scale", 0.01).value,
            Q=node.declare_parameter("dial.Q", [10, 10, 10, 5, 5, 5]).value,
            R=node.declare_parameter(
                "dial.R", [0.001, 0.01, 0.05]
            ).value,
            goal_check_horizon_s=node.declare_parameter(
                "dial.goal_check_horizon_s", 1.0
            ).value,
            goal_check_hz=node.declare_parameter(
                "dial.goal_check_hz", 25.0
            ).value,
        )
    

@dataclass
class MPCConfig:
    H_mpc: int = 6  # MPC horizon length in timesteps
    Q_mpc: list = None  # State cost weights for MPC
    R_mpc: list = None  # Control cost weights for MPC

    @classmethod
    def from_ros_node(cls, node: Node) -> "MPCConfig":
        if cls.Q_mpc is None:
            cls.Q_mpc = [10, 10, 10, 5, 5, 5]
        if cls.R_mpc is None:
            cls.R_mpc = [0.001, 0.01, 0.05]

        return cls(
            H_mpc=node.declare_parameter("mpc.H_mpc", 6).value,
            Q_mpc=node.declare_parameter("mpc.Q_mpc", [10, 10, 10, 5, 5, 5]).value,
            R_mpc=node.declare_parameter(
                "mpc.R_mpc", [0.001, 0.01, 0.05]
            ).value,
        )


@dataclass
class PathPlannerConfig:
    """Main configuration dataclass that aggregates all sub-configs."""
    table: TableConfig
    hovercraft: HovercraftConfig
    puck: PuckConfig
    dial: DIALConfig

    @classmethod
    def from_ros_node(cls, node: Node, id: Optional[int] = None) -> "PathPlannerConfig":
        return cls(
            table=TableConfig.from_ros_node(node),
            hovercraft=HovercraftConfig.from_ros_node(node, id),
            puck=PuckConfig.from_ros_node(node),
            dial=DIALConfig.from_ros_node(node),
        )


@dataclass
class ControlConfig:
    """Separate config for control node, if needed."""
    # For now we can just reuse the same config, but this allows for future separation if desired.
    table: TableConfig
    hovercraft: HovercraftConfig
    puck: PuckConfig
    mpc: MPCConfig

    @classmethod
    def from_ros_node(cls, node: Node, id: Optional[int] = None) -> "ControlConfig":
        return cls(
            table=TableConfig.from_ros_node(node),
            hovercraft=HovercraftConfig.from_ros_node(node, id),
            puck=PuckConfig.from_ros_node(node),
            mpc=MPCConfig.from_ros_node(node),
        )