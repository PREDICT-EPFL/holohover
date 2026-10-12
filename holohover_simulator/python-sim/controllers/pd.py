"""PD controller holding a robot at home (robot 1 in defence for the shot MPC, robot 2)."""

from functools import lru_cache

import jax
import jax.numpy as jnp


def build_pd_controller(target_state, robot_id: int):
    target_pos = target_state[:3]
    target_vel = target_state[3:6]

    if robot_id == 1:
        start_idx = 0
    else:
        start_idx = 6

    def controller(x0):
        pos = x0[start_idx:start_idx+3]
        vel = x0[start_idx+3:start_idx+6]
        Kp = 5.0
        Kd = 1.0
        pos_error = target_pos - pos
        vel_error = target_vel - vel

        u0 = Kp * pos_error + Kd * vel_error
        return u0

    return jax.jit(controller)


def home_state(config, robot_id):
    """Defensive position of robot `robot_id`, at rest: (x, y, theta, vx, vy, omega)."""
    rel = config["robot"][f"robot{robot_id}"]["defensive_position_rel"]
    return (float(rel * config["table"]["width"]), float(0.5 * config["table"]["height"]), 0.0, 0.0, 0.0, 0.0)


def defence_controller(config, robot_id):
    """PD controller holding robot `robot_id` at its defensive position, built once per process:
    every build_pd_controller call is a new jax.jit, which compiles on its first call (~20 ms)."""
    return _pd_controller(home_state(config, robot_id), robot_id)


@lru_cache(maxsize=None)
def _pd_controller(home, robot_id):
    return build_pd_controller(jnp.array(home), robot_id=robot_id)
