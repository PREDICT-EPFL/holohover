"""
Rollouts of DIAL's planning model (model.py) for robot 1, after Till's rollout.py: open loop
(rollout), under a state-feedback controller until the puck reaches an end wall
(rollout_controller), under the path follower toward a target (rollout_mpc), and a sampled input
sequence with its cost (rollout_dial).
"""

import jax
import jax.numpy as jnp

from .model import heading_to_goal, physics_step


def rollout(x0, control_seq, physics, dt, enemy_controller):
    """Robot 1 under control_seq, one input per step dt. Returns the states after each step and
    the robot 1 and robot 2 contact flags."""

    def step(x, u):
        x_next, r1_col, r2_col, _, _, _ = physics_step(x, jnp.concatenate([u, enemy_controller(x)]), physics, dt)
        return x_next, (x_next, r1_col, r2_col)

    _, (trajectory, r1_cols, r2_cols) = jax.lax.scan(step, x0, control_seq)
    return trajectory, r1_cols, r2_cols


def rollout_mpc(x0, target_pos, H, physics, dt, enemy_controller, follower):
    """H steps of dt with robot 1 driven by the path follower's final-state variant to target_pos
    (x, y, theta), reached at H dt with a speed of 2 sqrt(distance) toward it. Returns the states
    before each step and the inputs."""
    offset = target_pos[:2] - x0[:2]
    direction = offset / jnp.linalg.norm(offset + 1e-6)
    speed = 2 * jnp.sqrt(jnp.linalg.norm(offset))
    target_state = jnp.concatenate([target_pos, direction * speed, jnp.zeros(1)])

    def step(carry, _):
        x, k = carry
        u = jnp.clip(follower(x, target_state, H * dt, k * dt), -physics.robot.u_limits, physics.robot.u_limits)
        x_next, _, _, _, _, _ = physics_step(x, jnp.concatenate([u, enemy_controller(x)]), physics, dt)
        return (x_next, k + 1), (x, u)

    _, (trajectory, inputs) = jax.lax.scan(step, (x0, 0), length=H)
    return trajectory, inputs


def rollout_controller(x0, physics, H_max, dt, my_controller, enemy_controller):
    """
    Up to H_max steps of dt under state-feedback controllers, stopped once the puck reaches the
    left end wall. Used to check where a shot goes. Returns the states, whether robot 1 and
    robot 2 touch the puck, the goal flag, where the puck hits the left end wall (-1: it does
    not), the wall bounce sequence (wall indices 1-6 as decimal digits, first bounce last) and
    the step at which the puck reached the end wall (0 if it did not).
    """

    def step(carry, _):
        x, has_r1_col, has_r2_col, has_scored, hit_y, bounce_sequence, n_bounces = carry
        u = jnp.concatenate([my_controller(x), enemy_controller(x)])

        def stopped(_):
            return x, has_r1_col, has_r2_col, has_scored, hit_y, jnp.zeros(6)

        def moving(_):
            x_next, r1_col, r2_col, scored, hit_y_next, wall_bounces = physics_step(x, u, physics, dt)
            return x_next, r1_col, r2_col, scored, hit_y_next, wall_bounces

        x_next, r1_col, r2_col, has_scored, hit_y, wall_bounces = jax.lax.cond(
            hit_y != -1.0, stopped, moving, operand=None)

        walls = jnp.where(wall_bounces > 0, size=2, fill_value=-1)[0] + 1
        n_new = jnp.sum(wall_bounces > 0)
        bounce_sequence += jnp.where(n_new > 0, walls[0], 0) * 10**n_bounces
        bounce_sequence += jnp.where(n_new > 1, walls[1], 0) * 10 ** (n_bounces + 1)
        n_bounces += n_new

        has_r1_col = jnp.where(r1_col == 1, 1, has_r1_col)
        has_r2_col = jnp.where(r2_col == 1, 1, has_r2_col)
        return (x_next, has_r1_col, has_r2_col, has_scored, hit_y, bounce_sequence, n_bounces), (x, hit_y)

    (_, has_r1_col, has_r2_col, has_scored, hit_y, bounce_sequence, _), (trajectory, hit_ys) = jax.lax.scan(
        step, (x0, 0, 0, 0, -1.0, 0, 0), length=H_max)
    stopped_at = jnp.where(hit_y == -1.0, 0, jnp.argmax(hit_ys != -1.0))
    return trajectory, has_r1_col, has_r2_col, has_scored, hit_y, bounce_sequence, stopped_at


def rollout_dial(x0, input_seq, input_knots, tf, physics, H, H_goal_check, dt_goal, def_controller,
                 enemy_controller):
    """
    One sample of the planner: robot 1 under input_seq over tf (H steps), then robot 1 under
    def_controller until the puck reaches the left end wall. Returns the states, the cost and the
    wall bounce sequence of the shot, 67 when the shot is invalid (the puck misses the end wall or
    robot 2 touches it); the cost is inf when the puck is not heading to the goal at tf.
    """
    dt = tf / H
    half = 0.5 * physics.table.width

    def step(x, u):
        u = jnp.clip(u, -physics.robot.u_limits, physics.robot.u_limits)
        x_next, _, _, _, _, _ = physics_step(x, jnp.concatenate([u, enemy_controller(x)]), physics, dt)
        dist = jnp.linalg.norm(x[:2] - x[12:14])
        dist_next = jnp.linalg.norm(x_next[:2] - x_next[12:14])
        cost_approach = 1e2 * ((dist_next - dist) / tf) ** 3  # rewards closing in on the puck
        cost_side = jnp.where(x_next[0] < 0.52 * physics.table.width, 1e4 * (x_next[0] - half) ** 2, 0)
        return x_next, (x_next, cost_side + cost_approach)

    x_final, (trajectory, costs) = jax.lax.scan(step, x0, input_seq)

    # Inputs beyond the limits, and the jumps between knots
    knots = input_knots.reshape(-1)
    u_max = jnp.kron(jnp.ones(input_knots.shape[0]), physics.robot.u_limits)
    input_constraint_cost = 1e2 * (jnp.linalg.norm(jnp.minimum(knots, -u_max) + u_max) ** 2
                                   + jnp.linalg.norm(jnp.maximum(knots, u_max) - u_max) ** 2)
    input_continuity_cost = jnp.sum(jnp.diff(input_knots, axis=0) ** 2)

    final_position_cost = 4e2 * jnp.linalg.norm(x_final[:2] - x_final[12:14]) ** 2  # robot on the puck at tf
    tf_cost = 1e1 * tf**2
    base_cost = jnp.sum(costs) + input_constraint_cost + input_continuity_cost + final_position_cost + tf_cost

    def goal_check(_):
        post, _, will_r2_col, _, hit_y, bounce_sequence, stopped_at = rollout_controller(
            x_final, physics, H_goal_check, dt_goal, def_controller, enemy_controller)
        goal_cost = 1e3 * (hit_y - physics.table.left_goal[1]) ** 2
        # Rewards keeping the puck away from robot 2 until it reaches the end wall
        dist = jnp.linalg.norm(post[:, 12:14] - post[:, 6:8], axis=1)
        valid = jnp.arange(H_goal_check) < stopped_at
        n_valid = jnp.sum(valid)
        avg_dist = jnp.where(n_valid > 0, jnp.sum(jnp.where(valid, dist, 0.0)) / n_valid, 0.0)
        proximity_reward = -3e1 * avg_dist
        time_to_goal_cost = 3e1 * (stopped_at / H_goal_check) ** 2
        invalid = (hit_y == -1.0) | (will_r2_col == 1)
        return (base_cost + goal_cost + proximity_reward + time_to_goal_cost,
                jnp.where(invalid, 67, bounce_sequence))

    def reject(_):
        return jnp.inf, 67

    cost, bounce_sequence = jax.lax.cond(
        heading_to_goal(physics, x_final[12:14], x_final[15:17]), goal_check, reject, operand=None)
    return trajectory, cost, bounce_sequence
