"""Simple state-feedback controllers: a PD holding a robot at home, and DIAL's imagined opponent."""

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


def build_force_field_enemy_controller(home_state, table_width: float, robot_id: int):
    """Imagined opponent policy: home attraction + puck attraction + middle-line barrier."""
    home_pos = home_state[:2]
    mid_x = 0.5 * table_width

    side_sign = -1.0

    def controller(x0):
        robot_pos = x0[6:8]
        robot_vel = x0[9:11]
        puck_pos = x0[12:14]

        # Pull back to home while still reacting to puck position.
        k_home = 5.0
        k_puck = 0.0
        k_damp = 1.0

        home_force = k_home * (home_pos - robot_pos)

        # Project puck target to own half so imagined opponent does not chase across center.
        margin = 0.05 * table_width
        puck_x_limited = jnp.minimum(puck_pos[0], mid_x - margin)
        puck_target = jnp.array([puck_x_limited, puck_pos[1]])
        puck_force = k_puck * (puck_target - robot_pos)

        # Smooth repulsive barrier near the middle line.
        boundary_dist = side_sign * (robot_pos[0] - mid_x)
        barrier_strength = 12.0
        barrier = barrier_strength * jax.nn.softplus(-(boundary_dist - margin) / margin)
        barrier_force = jnp.array([side_sign * barrier, 0.0])

        damp_force = -k_damp * robot_vel

        u0_pos = home_force + puck_force + barrier_force + damp_force
        u0 = jnp.array([u0_pos[0], u0_pos[1], 0.0]) # no rotation command

        return u0

    return jax.jit(controller)
