"""
python-sim's side of the shot MPC's parameters and frames: the planner (holohover_planner) and the
controller (holohover_control) work in the table frame, the simulator in its world frame.

Parameters: the planner's and the controller's are those of the ROS nodes, in their YAML files
(holohover_utils/config/common): one config for both simulations. python-sim only puts its own
physics in (config.json: table, robot, puck).

World (simulator) frame: origin at the bottom-left corner, x in [0, width], y in [0, height].
Robot 1 plays on the right and attacks the left goal (x = 0).

Table frame (the ROS nodes'): origin at the table centre, axes as the world's:
    x_t = x_w - X/2,  y_t = y_w - W/2,  theta and velocities unchanged

Planner frame: x points from our goal to the opponent goal, wall L at y = +W/2, wall R at
y = -W/2. It is the world frame rotated by pi (holohover_planner Frame, attack_goal "left"):
    x_p = X - x_w,  y_p = W/2 - y_w,  v_p = -v_w,  theta_p = theta_w + pi
The world <-> planner maps below serve the recorded data and the tests.
"""

from pathlib import Path

import numpy as np

from holohover_planner.shot_mpc.params import ShotParams, load_parameters

# The ROS nodes' parameter files (src/holohover_utils/config/common)
CONFIG_DIR = Path(__file__).resolve().parents[4] / "holohover_utils" / "config" / "common"
PLANNER_CONFIG = CONFIG_DIR / "shot_planner_config.yaml"
CONTROLLER_CONFIG = CONFIG_DIR / "control_plan_config.yaml"
# python-sim's robot-puck collision is elastic (simulation.dynamics.robot_puck_collision),
# and its wall bounces are elastic (simulation.dynamics.wall_collision)
RESTITUTION = 1.0
WALL_RESTITUTION = 1.0

# Translational part (x, y, vx, vy) of a body state (x, y, theta, vx, vy, omega)
XY = [0, 1, 3, 4]


def planner_parameters(config):
    """The planner's parameters: shot_planner_config.yaml, with python-sim's table, robot and puck."""
    tbl, rob, pck = config["table"], config["robot"], config["puck"]
    p = load_parameters(PLANNER_CONFIG)
    p.update({
        "table.width": tbl["width"], "table.height": tbl["height"], "table.goal_width": tbl["goal_width"],
        "table.friction": tbl["friction"], "table.restitution": WALL_RESTITUTION,
        "robot.mass": rob["mass"], "robot.radius": rob["radius"], "robot.force_max": list(config["simulation"]["u_limits"]),
        "puck.mass": pck["mass"], "puck.radius": pck["radius"], "shot_mpc.restitution": RESTITUTION,
    })
    return p


def controller_parameters():
    """The controller's parameters: control_plan_config.yaml."""
    return load_parameters(CONTROLLER_CONFIG)


def shot_params(config):
    return ShotParams.from_parameters(planner_parameters(config))


def body_mass(P):
    """(m, m, I) of the robot: force (F_x, F_y, tau) = body_mass * acceleration (a_x, a_y, a_theta)."""
    return np.array([P.m_R, P.m_R, P.I_R])


def state_world_to_table(s, P: ShotParams):
    """(x, y, theta, vx, vy, omega) of a robot or the puck."""
    s = np.array(s, dtype=float)
    s[..., 0] -= 0.5 * P.X
    s[..., 1] -= 0.5 * P.W
    return s


def pos_table_to_world(p, P: ShotParams):
    p = np.array(p, dtype=float)
    p[..., 0] += 0.5 * P.X
    p[..., 1] += 0.5 * P.W
    return p


# The world <-> planner map is a rotation by pi, so each position/velocity map is its own inverse.
def pos_world_to_planner(p, P: ShotParams):
    p = np.asarray(p, dtype=float)
    return np.stack([P.X - p[..., 0], 0.5 * P.W - p[..., 1]], axis=-1)


pos_planner_to_world = pos_world_to_planner


def state_world_to_planner(s, P: ShotParams):
    s = np.asarray(s, dtype=float)
    return np.stack(
        [P.X - s[..., 0], 0.5 * P.W - s[..., 1], s[..., 2] + np.pi,
         -s[..., 3], -s[..., 4], s[..., 5]],
        axis=-1,
    )


def state_planner_to_world(s, P: ShotParams):
    s = np.asarray(s, dtype=float)
    return np.stack(
        [P.X - s[..., 0], 0.5 * P.W - s[..., 1], s[..., 2] - np.pi,
         -s[..., 3], -s[..., 4], s[..., 5]],
        axis=-1,
    )
