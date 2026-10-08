"""
Configuration object and the world <-> planner frame and unit conversions.

World (simulator) frame: origin at the bottom-left corner, x in [0, width],
y in [0, height]. Robot 1 plays on the right and attacks the left goal (x = 0).

Planner frame: x points from our goal to the opponent goal, wall L at
y = +W/2, wall R at y = -W/2. It is the world frame rotated by pi:
    x_p = X - x_w,  y_p = W/2 - y_w,  v_p = -v_w,  theta_p = theta_w + pi

The shot planner only uses the translational axes, (x, y, vx, vy) and (a_x, a_y);
the heading is the tracker's (Section 1).
"""

from dataclasses import dataclass

import numpy as np

# Translational part (x, y, vx, vy) of a body state (x, y, theta, vx, vy, omega)
XY = [0, 1, 3, 4]


@dataclass(frozen=True)
class ShotParams:
    # Physics, read from the simulator configuration
    X: float  # table length
    W: float  # table width
    h: float  # goal half-width
    gamma: float  # viscous friction, same for robot and puck
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
    jit: bool
    jit_flags: tuple

    @classmethod
    def from_config(cls, config):
        tbl, rob, pck = config["table"], config["robot"], config["puck"]
        c = config["shot_mpc"]
        wts, ip = c["weights"], c["ipopt"]
        return cls(
            X=tbl["width"],
            W=tbl["height"],
            h=0.5 * tbl["goal_width"],
            gamma=tbl["friction"],
            m_R=rob["mass"],
            I_R=0.5 * rob["mass"] * rob["radius"] ** 2,
            r_R=rob["radius"],
            m_p=pck["mass"],
            r_p=pck["radius"],
            F_max=tuple(config["simulation"]["u_limits"]),
            N=c["N"],
            t_min=c["t_min"],
            t_max=c["t_max"],
            K=c["K"],
            e=c["restitution"],
            goal_margin=c["goal_margin"],
            v_min=c["v_min"],
            eps=c["eps"],
            delta=c["delta"],
            t_clear=c["t_clear"],
            t_react=c["t_react"],
            t_ref=c["t_ref"],
            T_ref=c["T_ref"],
            w_u=wts["w_u"],
            w_du=wts["w_du"],
            w_t=wts["w_t"],
            w_T=wts["w_T"],
            w_a=wts["w_a"],
            w_slack=wts["w_slack"],
            outer_hz=c["rates"]["outer_hz"],
            inner_hz=c["rates"]["inner_hz"],
            t_commit=c["t_commit"],
            t_stop_replan=c["t_stop_replan"],
            plan_expiry=c["plan_expiry"],
            contact_cooldown=c["contact_cooldown"],
            sigma_init=c["sigma_init"],
            n_tf_seeds=c["n_tf_seeds"],
            max_iter=ip["max_iter"],
            tol=ip["tol"],
            acceptable_tol=ip["acceptable_tol"],
            valid_tol=ip["valid_tol"],
            jit=c["jit"],
            jit_flags=tuple(c["jit_flags"]),
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
# The map is a rotation by pi, so each position/velocity map is its own inverse.


def pos_world_to_planner(p, P: ShotParams):
    p = np.asarray(p, dtype=float)
    return np.stack([P.X - p[..., 0], 0.5 * P.W - p[..., 1]], axis=-1)


pos_planner_to_world = pos_world_to_planner


def state_world_to_planner(s, P: ShotParams):
    """(x, y, theta, vx, vy, omega) of a robot or the puck."""
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


def accel_planner_to_force_world(u, P: ShotParams):
    """Planner accelerations (a_x, a_y, a_theta) -> simulator input (F_x, F_y, tau)."""
    u = np.asarray(u, dtype=float)
    return np.stack([-P.m_R * u[..., 0], -P.m_R * u[..., 1], P.I_R * u[..., 2]], axis=-1)


def force_world_to_accel_planner(F, P: ShotParams):
    F = np.asarray(F, dtype=float)
    return np.stack([-F[..., 0] / P.m_R, -F[..., 1] / P.m_R, F[..., 2] / P.I_R], axis=-1)
