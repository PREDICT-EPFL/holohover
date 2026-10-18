"""
Configuration object (from ROS parameters) and the table <-> planner frame transform.

Planner frame: x points from our goal (x = 0) to the opponent goal (x = X), wall L at
y = +W/2, wall R at y = -W/2.

The shot planner only uses the translational axes, (x, y, vx, vy) and (a_x, a_y);
the heading is the tracker's (Section 1).
"""

from dataclasses import dataclass

import numpy as np


def load_parameters(path):
    """The ros__parameters of a node's YAML parameter file, keyed by their dotted names (shot_mpc.N, ...):
    the planner's values for the tests and the tools that run it without ROS."""
    import yaml

    def flat(tree, prefix=""):
        out = {}
        for key, value in tree.items():
            out.update(flat(value, f"{prefix}{key}.") if isinstance(value, dict) else {prefix + key: value})
        return out

    with open(path) as f:
        (params,) = (node["ros__parameters"] for node in yaml.safe_load(f).values())
    return flat(params)


# ShotParams field -> its parameter under shot_mpc.
_SHOT_MPC_KEYS = {
    "N": "N", "t_min": "t_min", "t_max": "t_max", "K": "K", "e": "restitution", "goal_margin": "goal_margin",
    "v_min": "v_min", "eps": "eps", "delta": "delta", "t_clear": "t_clear", "t_react": "t_react",
    "t_ref": "t_ref", "T_ref": "T_ref",
    "w_u": "weights.w_u", "w_du": "weights.w_du", "w_t": "weights.w_t", "w_T": "weights.w_T",
    "w_a": "weights.w_a", "w_slack": "weights.w_slack",
    "outer_hz": "rates.outer_hz", "inner_hz": "rates.inner_hz", "t_commit": "t_commit",
    "t_stop_replan": "t_stop_replan", "plan_expiry": "plan_expiry", "contact_cooldown": "contact_cooldown",
    "sigma_init": "sigma_init", "n_tf_seeds": "n_tf_seeds",
    "max_iter": "ipopt.max_iter", "tol": "ipopt.tol", "acceptable_tol": "ipopt.acceptable_tol",
    "valid_tol": "ipopt.valid_tol",
}

# The planner's ROS parameters (ShotParams), by name. Their values are only in
# holohover_utils/config/common/shot_planner_config.yaml: no defaults in the code.
PARAMETERS = (
    "table.width", "table.height", "table.goal_width", "table.friction", "table.restitution",
    "robot.mass", "robot.radius", "robot.force_max", "puck.mass", "puck.radius",
    *("shot_mpc." + key for key in _SHOT_MPC_KEYS.values()),
)


@dataclass(frozen=True)
class ShotParams:
    # Physics
    X: float  # table length
    W: float  # table width
    h: float  # goal half-width
    gamma: float  # viscous friction, same for robot and puck
    e_w: float  # wall restitution: a puck bounce reverses the normal velocity scaled by e_w, keeps the tangential one
    m_R: float
    I_R: float
    r_R: float
    m_p: float
    r_p: float
    F_max: tuple  # (Fx, Fy, tau) limits
    # Planner
    N: int
    t_min: float
    t_max: float
    K: int
    e: float  # robot-puck restitution
    goal_margin: float
    v_min: float
    eps: float
    delta: float
    t_clear: float  # post-impact wall clearance time of the robot (0 disables)
    t_react: float  # post-impact delay before the robot brakes, for the midline stop row
    t_ref: float
    T_ref: float
    # Weights of the cost terms A-E (Section 5) and of the soft-constraint slacks
    w_u: float
    w_du: float
    w_t: float
    w_T: float
    w_a: float
    w_slack: float
    # Closed loop
    outer_hz: float
    inner_hz: float
    t_commit: float
    t_stop_replan: float
    plan_expiry: float
    contact_cooldown: float
    sigma_init: float
    n_tf_seeds: int
    # Solver
    max_iter: int
    tol: float
    acceptable_tol: float
    valid_tol: float

    @classmethod
    def from_parameters(cls, p):
        """From the parameter values, keyed by the names of PARAMETERS."""
        return cls(
            X=p["table.width"],
            W=p["table.height"],
            h=0.5 * p["table.goal_width"],
            gamma=p["table.friction"],
            e_w=p["table.restitution"],
            m_R=p["robot.mass"],
            I_R=0.5 * p["robot.mass"] * p["robot.radius"] ** 2,
            r_R=p["robot.radius"],
            m_p=p["puck.mass"],
            r_p=p["puck.radius"],
            F_max=tuple(p["robot.force_max"]),
            **{name: p["shot_mpc." + key] for name, key in _SHOT_MPC_KEYS.items()},
        )

    # Effective geometry for the puck centre (Section 1)
    @property
    def w(self):
        return 0.5 * self.W - self.r_p

    @property
    def x_g(self):
        return self.X - self.r_p

    @property
    def x_mid(self):
        return 0.5 * self.X

    @property
    def h_eff(self):
        return self.h - self.r_p - self.goal_margin

    @property
    def kappa(self):
        return (1.0 + self.e) * self.m_R / (self.m_R + self.m_p)

    @property
    def u_max(self):
        """Acceleration limits (a_x, a_y, a_theta); the planner uses the first two."""
        return np.array([self.F_max[0] / self.m_R, self.F_max[1] / self.m_R, self.F_max[2] / self.I_R])


###########################
### Frame transforms    ###
###########################
# Table frame: origin at the table centre, x along the length. The planner attacks the goal at
# x = -X/2 (attack_goal "left", python-sim's robot 1) or at x = +X/2 ("right"):
#     left:   x_p = X/2 - x,  y_p = -y,  v_p = -v   (rotation by pi)
#     right:  x_p = X/2 + x,  y_p = y,   v_p = v    (translation)


class Frame:
    def __init__(self, P: ShotParams, attack_goal="left"):
        if attack_goal not in ("left", "right"):
            raise ValueError(f"attack_goal {attack_goal!r}: use left or right")
        self.sign = -1.0 if attack_goal == "left" else 1.0
        self.offset = np.array([0.5 * P.X, 0.0])

    def pos_to_planner(self, p):
        return self.offset + self.sign * np.asarray(p, dtype=float)

    def pos_to_table(self, p):
        return self.sign * (np.asarray(p, dtype=float) - self.offset)

    def vec_to_planner(self, v):
        """Velocities and accelerations."""
        return self.sign * np.asarray(v, dtype=float)

    vec_to_table = vec_to_planner
