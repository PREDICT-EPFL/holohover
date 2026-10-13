"""
DIAL's planning model: the physics of Till's simulator (non-rt-simulation/dynamics.py), with
which the planner rolls out its samples and the path follower predicts. It differs from the
plant (simulation/dynamics.py) in how a body is integrated over a step: positions also advance
by half the acceleration (x += v dt + a dt^2 / 2) and the torque turns the robot. Walls and
robot-puck collisions are the plant's. Forces are clipped to robot.u_limits, as in the plant.
"""

import jax
import jax.numpy as jnp

from simulation.dynamics import robot_puck_collision, wall_collision


def discrete_system(physics, dt):
    """(A_d, B_d) of one robot over dt, as integrate_body steps it (no walls, no collisions)."""
    drag = 1.0 - physics.table.friction * dt
    Ad = jnp.block([[jnp.eye(3), dt * jnp.eye(3)], [jnp.zeros((3, 3)), drag * jnp.eye(3)]])
    M = jnp.diag(jnp.array([1 / physics.robot.mass, 1 / physics.robot.mass, 1 / physics.robot.inertia]))
    Bd = jnp.block([[0.5 * dt**2 * M], [dt * M]])
    return Ad, Bd


def heading_to_goal(physics, puck_pos, puck_vel):
    """Whether the puck is heading to the left goal (robot 1's target), directly or over the side
    walls: its straight line, unfolded across the side walls, ends within the goal."""
    table = physics.table
    goal_y_min = 0.5 * table.height - 0.5 * table.goal_width
    goal_y_max = 0.5 * table.height + 0.5 * table.goal_width

    a = puck_vel[1] / (puck_vel[0] + 1e-8)  # line y = a x + b
    b = puck_pos[1] - a * puck_pos[0]
    y_hit = a * physics.puck.radius + b
    # Wall bounces seen as a move to a virtual table next to the real one
    y_hit = y_hit % (table.height - 2 * physics.puck.radius)
    return (y_hit >= goal_y_min) & (y_hit <= goal_y_max) & (puck_vel[0] < 0)


def integrate_body(state, force, entity, table, dt):
    pos, vel = state[:3], state[3:]
    acc = force / jnp.array([entity.mass, entity.mass, entity.inertia])
    pos = pos + vel * dt + 0.5 * acc * dt**2
    vel = vel * (1 - table.friction * dt) + acc * dt
    pos, vel, has_scored, hit_y, wall_bounces = wall_collision(pos, vel, entity.radius, table)
    return jnp.concatenate([pos, vel]), has_scored, hit_y, wall_bounces


@jax.jit
def physics_step(state, control, physics, dt):
    """One step of the 18-dim state [robot1, robot2, puck] under control [robot1 (3), robot2 (3)].
    Same outputs as simulation.dynamics.physics_step, with the contact flags as 0/1 integers:
    state_next, r1_has_collision, r2_has_collision, has_scored, hit_y (where the puck hits the
    left end wall, -1 if it does not), wall_bounces."""
    u_max = physics.robot.u_limits
    r1, _, _, _ = integrate_body(state[:6], jnp.clip(control[:3], -u_max, u_max), physics.robot, physics.table, dt)
    r2, _, _, _ = integrate_body(state[6:12], jnp.clip(control[3:6], -u_max, u_max), physics.robot, physics.table, dt)
    puck, has_scored, hit_y, wall_bounces = integrate_body(state[12:18], jnp.zeros(3), physics.puck, physics.table, dt)

    # Robot 1 then robot 2 against the puck: only one robot can hit it per step
    r1_pos, r1_vel, p_pos, p_vel, r1_col = robot_puck_collision(
        r1[:2], r1[3:5], puck[:2], puck[3:5], physics.robot, physics.puck)
    r1 = r1.at[:2].set(r1_pos).at[3:5].set(r1_vel)
    r2_pos, r2_vel, p_pos, p_vel, r2_col = robot_puck_collision(
        r2[:2], r2[3:5], p_pos, p_vel, physics.robot, physics.puck)
    r2 = r2.at[:2].set(r2_pos).at[3:5].set(r2_vel)
    puck = puck.at[:2].set(p_pos).at[3:5].set(p_vel)

    return (jnp.concatenate([r1, r2, puck]), jnp.asarray(r1_col, dtype=jnp.int32),
            jnp.asarray(r2_col, dtype=jnp.int32), has_scored, hit_y, wall_bounces)
