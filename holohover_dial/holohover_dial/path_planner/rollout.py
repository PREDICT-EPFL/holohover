import jax
import jax.numpy as jnp

from controller import build_mpc_controller, build_pd_controller
from dynamics import physics_step


def print_cost_overview(
    stage_cost,
    input_cost,
    tf_cost,
    goal_cost,
    proximity_reward,
    total_cost,
    hit_y,
    will_score,
    will_r2p_collision,
    stopped_at,
):
    jax.debug.print(
        (
            "\nDIAL cost overview | hit_y={hit_y:.3f} | stage={stage_cost:.3f} | "
            "tf_cost={tf_cost:.3f} | input_cost={input_cost:.3f} | "
            "goal={goal_cost:.3f} | "
            "proximity_reward={proximity_reward:.3f} | total={total_cost:.3f} | "
            "score={will_score} | r2_collision_flag={will_r2p_collision} | stopped_at={stopped_at}"
        ),
        hit_y=hit_y,
        stage_cost=stage_cost,
        input_cost=input_cost,
        tf_cost=tf_cost,
        goal_cost=goal_cost,
        proximity_reward=proximity_reward,
        total_cost=total_cost,
        will_score=will_score,
        will_r2p_collision=will_r2p_collision,
        stopped_at=stopped_at,
    )
    return 0


def rollout(x0, control_seq, physics, dt, robot_id, enemy_controller):
    """
    Rollout from a given starting position using a pre-defined set of inputs at refresh rate dt.

    This is used
    1. to visualize Path Plan when DIAL has finished cooking up an input sequence
    2. to initilize the simulation by providing predicted future puck position as target for a PD controller
    """

    def step(x, u):
        u0_me = u
        u0_enemy = enemy_controller(x)
        u0_me = jnp.clip(u0_me, -physics.robot.u_limits, physics.robot.u_limits)
        u0_enemy = jnp.clip(u0_enemy, -physics.robot.u_limits, physics.robot.u_limits)
        u0 = jnp.where(robot_id==1, jnp.concatenate([u0_me, u0_enemy]), jnp.concatenate([u0_enemy, u0_me]))

        x_next, r1_has_collision, r2_has_collision, _, _, _, _ = physics_step(x, u0, physics, dt)
        
        return x_next, (x_next, r1_has_collision, r2_has_collision)

    _, (trajectory, r1_collisions_detected, r2_collisions_detected) = jax.lax.scan(step, x0, control_seq)
    return trajectory, r1_collisions_detected, r2_collisions_detected


def rollout_pd(x0, target_pos, H, physics, dt, robot_id, enemy_controller):
    """
    Rollout from a given starting position using a PD-controller tracking a fixed target state.

    Used to warm-start simulation (robot set to track future puck position)
    """
    start_idx = 0 if robot_id==1 else 6

    # achieve max velocity in direction of target, proportional to distance
    target_direction = (target_pos[:2] - x0[start_idx:start_idx+2]) / jnp.linalg.norm(
        target_pos[:2] - x0[start_idx:start_idx+2] + 1e-6
    )
    target_speed = 4.5 * jnp.sqrt(jnp.linalg.norm(target_pos[:2] - x0[start_idx:start_idx+2]))
    target_vel = jnp.concatenate([target_direction * target_speed, jnp.zeros(1)])

    pd_controller = build_pd_controller(
        jnp.concatenate([target_pos, target_vel]), robot_id)

    def step(x, _):
        u0_me = pd_controller(x)
        u0_enemy = enemy_controller(x)
        u0_me = jnp.clip(u0_me, -physics.robot.u_limits, physics.robot.u_limits)
        u0_enemy = jnp.clip(u0_enemy, -physics.robot.u_limits, physics.robot.u_limits)
        u0 = jnp.where(robot_id==1, jnp.concatenate([u0_me, u0_enemy]), jnp.concatenate([u0_enemy, u0_me]))

        x_next, _, _, _, _, _, _ = physics_step(x, u0, physics, dt)
        return x_next, (x, u0_me)

    _, (trajectory, inputs) = jax.lax.scan(step, x0, length=H)
    return trajectory, inputs


def rollout_mpc(x0, target_pos, H, physics, dt, robot_id, enemy_controller, config):
    """
    Rollout from a given starting position using a MPC controller tracking a fixed target state.

    Used to warm-start simulation (robot set to track future puck position)
    """
    start_idx = 0 if robot_id==1 else 6

    # achieve max velocity in direction of target, proportional to distance
    target_direction = (target_pos[:2] - x0[start_idx:start_idx+2]) / jnp.linalg.norm(
        target_pos[:2] - x0[start_idx:start_idx+2] + 1e-6
    )
    target_speed = 2 * jnp.sqrt(jnp.linalg.norm(target_pos[:2] - x0[start_idx:start_idx+2]))
    target_vel = jnp.concatenate([target_direction * target_speed, jnp.zeros(1)])
    # target_vel = jnp.zeros(3)  # for pure position tracking

    mpc_controller = build_mpc_controller(config, physics, robot_id, final_only=True)

    target_state = jnp.concatenate([target_pos, target_vel])

    def step(carry, _):
        x, iter = carry
        u0_me = mpc_controller(x, target_state, H*dt, iter*dt)
        u0_enemy = enemy_controller(x)
        u0_me = jnp.clip(u0_me, -physics.robot.u_limits, physics.robot.u_limits)
        u0_enemy = jnp.clip(u0_enemy, -physics.robot.u_limits, physics.robot.u_limits)
        u0 = jnp.where(robot_id==1, jnp.concatenate([u0_me, u0_enemy]), jnp.concatenate([u0_enemy, u0_me]))

        x_next, _, _, _, _, _, _ = physics_step(x, u0, physics, dt)
        return (x_next, iter+1), (x, u0_me)

    _, (trajectory, inputs) = jax.lax.scan(step, (x0, 0), length=H)
    return trajectory, inputs


def rollout_controller(x0, physics, H_max, dt, robot_id, my_controller, enemy_controller):
    """
    Rollout from a given initial state with a given controller (defined as u = f(state)).
    Simulate only until a goal is scored.

    Used in DIAL cost calulation to see if shot went in.
    """

    def step(carry, _):
        (
            x,
            has_r1p_collision,
            has_r2p_collision,
            has_scored,
            hit_y,
            wall_bounce_sequence,
            bounce_idx,
        ) = carry
        u0_me = my_controller(x)
        u0_enemy = enemy_controller(x)
        u0_me = jnp.clip(u0_me, -physics.robot.u_limits, physics.robot.u_limits)
        u0_enemy = jnp.clip(u0_enemy, -physics.robot.u_limits, physics.robot.u_limits)
        u0 = jnp.where(robot_id==1, jnp.concatenate([u0_me, u0_enemy]), jnp.concatenate([u0_enemy, u0_me]))

        def scored_branch(_):  # do nothing
            return (
                x,
                has_r1p_collision,
                has_r2p_collision,
                has_scored,
                hit_y,
                jnp.zeros(6),
            )

        def normal_branch(_):
            # Regular physics step
            (
                x_next,
                r1p_collision_next,
                r2p_collision_next,
                has_scored_next,
                hit_y_left_next,
                hit_y_right_next,
                wall_bounces_next,
            ) = physics_step(x, u0, physics, dt)
            hit_y_next = jnp.where(robot_id==1, hit_y_left_next, hit_y_right_next)
            return (
                x_next,
                r1p_collision_next,
                r2p_collision_next,
                has_scored_next,
                hit_y_next,
                wall_bounces_next,
            )

        hit_wall = hit_y != -1.0  # magic number defined in dynamics
        trajectory, r1p_collision, r2p_collision, has_scored, hit_y, wall_bounces = (
            jax.lax.cond(hit_wall, scored_branch, normal_branch, operand=None)
        )

        # Extract wall indices from wall bounces
        wall_indices = jnp.where(wall_bounces > 0, size=2, fill_value=-1)[0] + 1
        n_bounces = jnp.sum(wall_bounces > 0)

        # Store
        wall_bounce_sequence += jnp.where(n_bounces > 0, wall_indices[0], 0) * (
            10**bounce_idx
        )
        wall_bounce_sequence += jnp.where(n_bounces > 1, wall_indices[1], 0) * (
            10 ** (bounce_idx + 1)
        )
        bounce_idx += n_bounces

        # Register and remember Robot-Puck collisions
        has_r1p_collision = jnp.where(r1p_collision == 1, 1, has_r1p_collision)
        has_r2p_collision = jnp.where(r2p_collision == 1, 1, has_r2p_collision)

        return (
            trajectory,
            has_r1p_collision,
            has_r2p_collision,
            has_scored,
            hit_y,
            wall_bounce_sequence,
            bounce_idx,
        ), (x, hit_y)

    (
        _,
        has_r1p_collision,
        has_r2p_collision,
        has_scored,
        hit_y,
        wall_bounce_sequence,
        _,
    ), (
        trajectory,
        hit_y_sequence,
    ) = jax.lax.scan(
        step, (x0, 0, 0, 0, -1.0, 0, 0), length=H_max
    )

    sim_stopped_idx = jnp.where(
        hit_y == -1.0,
        0,
        jnp.argmax(hit_y_sequence != -1.0),
    )

    return (
        trajectory,
        has_r1p_collision,
        has_r2p_collision,
        has_scored,
        hit_y,
        wall_bounce_sequence,
        sim_stopped_idx,  # stopping sim when puck hits goal wall
    )


def rollout_dial(
    x0,
    input_seq,
    input_knots,
    sample_idx,
    tf,
    R,
    physics,
    H,
    H_goal_check,
    dt_sim,
    robot_id,
    my_def_controller,
    enemy_controller,
):
    """
    Rollout a starting position with given inputs while computing the costs.
    Used inside the DIAL function to evaluate cost of all randomly generate trajectories.
    """
    dt = tf / H
    # dt = jnp.clip(dt, 1e-3, None)  # prevent exploding when tf is very small

    def step(carry, u):
        x, u_prev = carry

        u0_me = u
        u0_enemy = enemy_controller(x)
        u0_me = jnp.clip(u0_me, -physics.robot.u_limits, physics.robot.u_limits)
        u0_enemy = jnp.clip(u0_enemy, -physics.robot.u_limits, physics.robot.u_limits)
        u0 = jnp.where(robot_id==1, jnp.concatenate([u0_me, u0_enemy]), jnp.concatenate([u0_enemy, u0_me]))

        x_next, _, _, _, _, _, _ = physics_step(x, u0, physics, dt)
        r_pos = jnp.where(robot_id==1, x[:2], x[6:8])
        r_pos_next = jnp.where(robot_id==1, x_next[:2], x_next[6:8])

        puck_dist_prev = jnp.linalg.norm(r_pos - x[12:14])
        puck_dist_next = jnp.linalg.norm(r_pos_next - x_next[12:14])
        cost_approach_puck = 1e2 * ((puck_dist_next - puck_dist_prev) / tf) ** 3
        # cost_approach_puck = 0

        # cost_side = jnp.where(r1_pos[0] <= 0.5 * physics.table.width, 1e10, 0)
        wrong_side_cond = jnp.where(robot_id==1, r_pos_next[0] < 0.52 * physics.table.width, r_pos_next[0] > 0.48 * physics.table.width)
        cost_side = jnp.where(
            wrong_side_cond,
            1e4 * (r_pos_next[0] - 0.5 * physics.table.width) ** 2,
            0,
        )
        # cost_side = 0

        cost = cost_side + cost_approach_puck
        return (x_next, u), (x_next, cost)

    carry_init = (x0, jnp.zeros_like(input_seq[0]))
    (x_final, _), (trajectory, costs) = jax.lax.scan(
        step, carry_init, input_seq
    )

    ### Check if puck heading to goal, if not reject immediately
    p_pos = x_final[12:14]
    p_vel = x_final[15:17]

    heading_to_goal = physics.check_puck_heading_goal(robot_id, p_pos, p_vel)

    ###

    stage_cost = jnp.sum(costs)

    # Input costs 1/2 * u^T R u, R = lambda * Sigma^-1 on knot variables.
    u = input_knots.reshape(-1)
    input_cost = 0.5 * u.T @ R @ u * 1e-1

    # punish going outside of input limits with u[0] and u[1]
    K = input_knots.shape[0]  # number of knots
    u_max = jnp.kron(jnp.ones(K), jnp.array(physics.robot.u_limits))
    input_constraint_cost = jnp.linalg.norm(jnp.minimum(u, -u_max) + u_max) ** 2
    input_constraint_cost += jnp.linalg.norm(jnp.maximum(u, u_max) - u_max) ** 2
    input_constraint_cost *= 1e2

    # Punish non smoothness of input sequence
    input_diff = jnp.diff(input_knots, axis=0)
    input_continuity_cost = jnp.sum(input_diff**2) * 1e0

    # Final position cost
    r_pos = jnp.where(robot_id==1, x_final[:2], x_final[6:8])
    p_pos = x_final[12:14]
    final_position_cost = jnp.where(tf > 0.101, 4e2 * jnp.linalg.norm(r_pos - p_pos) ** 2, 0)

    # Minimize time to hit puck
    tf_cost = 1e1 * tf**2

    base_cost = (
        stage_cost
        # + input_cost
        + input_constraint_cost
        + input_continuity_cost
        + final_position_cost
        + tf_cost
    )

    def compute_goal_check_cost(_):
        # Rollout again only for promising trajectories to see if puck reaches goal.
        (
            post_traj,
            will_r1p_collision,
            will_r2p_collision,
            _,
            hit_y,
            wall_bounce_sequence,
            stopped_at,
        ) = rollout_controller(
            x_final,
            physics,
            H_goal_check,
            dt_sim,
            robot_id,
            my_def_controller,
            enemy_controller,
        )

        enemy_goal_pos = jnp.where(robot_id==1, physics.table.left_goal, physics.table.right_goal)
        goal_cost = 1e3 * (hit_y - enemy_goal_pos[1]) ** 2

        puck_pos_post = post_traj[:, 12:14]
        enemy_pos_post = jnp.where(robot_id==1, post_traj[:, 6:8], post_traj[:, 0:2])
        dist_post = jnp.linalg.norm(puck_pos_post - enemy_pos_post, axis=1)
        valid_mask = jnp.arange(H_goal_check) < stopped_at
        masked_dist_post = jnp.where(valid_mask, dist_post, 0.0)
        valid_count = jnp.sum(valid_mask)

        avg_dist_post = jnp.where(
            valid_count > 0,
            jnp.sum(masked_dist_post) / valid_count,
            0.0,
        )
        proximity_reward = -3e1 * avg_dist_post

        time_to_goal_cost = 3e1 * (stopped_at / H_goal_check) ** 2

        total_cost = base_cost + goal_cost + proximity_reward + time_to_goal_cost

        # Not a 67 joke: Disqualify trajectories with invalid wall sequence.
        no_goal = hit_y == -1.0
        enemy_intercept = jnp.where(robot_id==1, will_r2p_collision == 1, will_r1p_collision == 1)
        wall_bounce_sequence = jnp.where(
            no_goal | enemy_intercept, 67, wall_bounce_sequence
        )
        return total_cost, wall_bounce_sequence

    def reject_trajectory(_):
        return jnp.inf, 67

    total_cost, wall_bounce_sequence = jax.lax.cond(
        heading_to_goal,
        compute_goal_check_cost,
        reject_trajectory,
        operand=None,
    )

    return trajectory, total_cost, wall_bounce_sequence
