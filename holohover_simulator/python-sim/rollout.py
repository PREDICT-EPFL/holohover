import jax
import jax.numpy as jnp

from dynamics import physics_step


def print_cost_overview(
    stage_cost,
    positive_tf_cost,
    enforce_collision_cost,
    goal_cost,
    enemy_robot_collision_cost,
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
            "tf_positive={positive_tf_cost:.3f} | enforce_collision={enforce_collision_cost:.3f} | "
            "goal={goal_cost:.3f} | r2_collision={enemy_robot_collision_cost:.3f} | "
            "proximity_reward={proximity_reward:.3f} | total={total_cost:.3f} | "
            "score={will_score} | r2_collision_flag={will_r2p_collision} | stopped_at={stopped_at}"
        ),
        hit_y=hit_y,
        stage_cost=stage_cost,
        positive_tf_cost=positive_tf_cost,
        enforce_collision_cost=enforce_collision_cost,
        goal_cost=goal_cost,
        enemy_robot_collision_cost=enemy_robot_collision_cost,
        proximity_reward=proximity_reward,
        total_cost=total_cost,
        will_score=will_score,
        will_r2p_collision=will_r2p_collision,
        stopped_at=stopped_at,
    )
    return 0


def rollout(x0, control_seq, physics, dt, r2_controller):
    """
    Rollout from a given starting position using a pre-defined set of inputs at refresh rate dt.

    This is used
    1. to visualize Path Plan when DIAL has finished cooking up an input sequence
    2. to initilize the simulation by prodiding predicted future puck position as target for a PD controller
    """

    def step(x, u):
        u0_r1 = u
        u0_r2 = r2_controller(x)
        u0 = jnp.concatenate([u0_r1, u0_r2])
        x_next, r1_has_collision, _, _, _, _ = physics_step(x, u0, physics, dt)
        return x_next, (x_next, r1_has_collision)

    _, (trajectory, r1_collisions_detected) = jax.lax.scan(step, x0, control_seq)
    return trajectory, r1_collisions_detected


def rollout_pd(x0, target_pos, H, physics, dt, r2_controller):
    """
    Rollout from a given starting position using a PD-controller tracking a fixed target state.

    Used to warm-start simulation (robot set to track future puck position)
    """

    def step(x, _):
        kp = 10
        kd = 2
        u0_r1 = kp * (target_pos - x[:3]) + kd * (jnp.zeros(3) - x[3:6])
        u0_r2 = r2_controller(x)
        u0 = jnp.concatenate([u0_r1, u0_r2])
        x_next, _, _, _, _, _ = physics_step(x, u0, physics, dt)
        return x_next, (x, u0_r1)

    _, (trajectory, inputs) = jax.lax.scan(step, x0, length=H)
    return trajectory, inputs


def rollout_controller(x0, physics, H_max, dt, r1_controller, r2_controller):
    """
    Rollout from a given initial state with a given controller (defined as u = f(state)).
    Simulate only until a goal is scored.

    Used in DIAL cost calulation to see if shot went in.
    """

    def step(carry, _):
        x, has_r2p_collision, has_scored, hit_y, wall_bounce_sequence, bounce_idx = (
            carry
        )
        u0_r1 = r1_controller(x)
        u0_r2 = r2_controller(x)
        u0 = jnp.concatenate([u0_r1, u0_r2])

        def scored_branch(_):  # do nothing
            return x, 0, False, has_scored, hit_y, jnp.zeros(6)

        def normal_branch(_):
            # Regular physics step
            (
                x_next,
                _,
                r2p_collision_next,
                has_scored_next,
                hit_y_next,
                wall_bounces_next,
            ) = physics_step(x, u0, physics, dt)
            return (
                x_next,
                0,
                r2p_collision_next,
                has_scored_next,
                hit_y_next,
                wall_bounces_next,
            )

        hit_wall = hit_y != -1.0  # magic number defined in dynamics
        trajectory, _, r2p_collision, has_scored, hit_y, wall_bounces = jax.lax.cond(
            hit_wall, scored_branch, normal_branch, operand=None
        )

        # Extract wall indices from wall bounces
        wall_indices = jnp.where(wall_bounces > 0, size=2, fill_value=-1)[0]
        has_bounce = jnp.any(wall_bounces > 0)
        # Store
        wall_bounce_sequence = wall_bounce_sequence.at[bounce_idx].set(
            jnp.where(has_bounce, wall_indices[0], -1)
        )
        wall_bounce_sequence = wall_bounce_sequence.at[bounce_idx + 1].set(
            jnp.where(has_bounce & (wall_indices[1] != -1), wall_indices[1], -1)
        )
        bounce_idx = bounce_idx + jnp.where(has_bounce, jnp.sum(wall_bounces > 0), 0)

        # Register and remember Robot2-Puck collision
        has_r2p_collision = jnp.where(r2p_collision == 1, 1, has_r2p_collision)

        return (
            trajectory,
            has_r2p_collision,
            has_scored,
            hit_y,
            wall_bounce_sequence,
            bounce_idx,
        ), (x, hit_y)

    wall_bounce_sequence = jnp.full(
        10, -1, dtype=jnp.int32
    )  # Pre-allocate with -1 as padding, also very naughty hardcoding
    (_, has_r2p_collision, has_scored, hit_y, wall_bounce_sequence, _), (
        trajectory,
        hit_y_sequence,
    ) = jax.lax.scan(step, (x0, 0, 0, -1.0, wall_bounce_sequence, 0), length=H_max)

    sim_stopped_idx = jnp.where(
        hit_y == -1.0, H_max - 1, jnp.argmax(hit_y_sequence != -1.0)
    )

    return (
        trajectory,
        has_r2p_collision,
        has_scored,
        hit_y,
        wall_bounce_sequence,
        sim_stopped_idx,
    )


def rollout_dial(
    x0,
    input_seq,
    sample_idx,
    tf,
    Q,
    R,
    physics,
    H,
    H_goal_check,
    dt_sim,
    def_controller,
    r2_controller,
):
    """
    Rollout a starting position with given inputs while computing the costs.
    Used inside the DIAL function to evaluate cost of all randomly generate trajectories.
    """
    dt = tf / H
    # dt = jnp.clip(dt, 1e-3, None)  # prevent exploding when tf is very small

    def step(carry, u):
        x, u_prev = carry

        u0_r1 = u
        u0_r2 = r2_controller(x)
        u0 = jnp.concatenate([u0_r1, u0_r2])

        x_next, r1_has_collision, _, _, _, _ = physics_step(x, u0, physics, dt)
        r1_pos = x_next[:2]
        p_pos = x_next[12:14]

        cost_du = 1e3 * (u - u_prev) @ R @ (u - u_prev)
        cost_du = 0

        cost_u = 1e0 * u @ R @ u
        # cost_u = 0

        cost_dist = 1e1*jnp.linalg.norm(r1_pos - p_pos) ** 2
        # cost_dist = 0

        cost_side = jnp.where(r1_pos[0] <= 0.5 * physics.table.width, 1e10, 0)
        # cost_side = 0

        cost = cost_side + cost_du + cost_dist + cost_u
        return (x_next, u), (x_next, cost, r1_has_collision)

    carry_init = (x0, jnp.zeros_like(input_seq[0]))
    (x_final, _), (trajectory, costs, puck_collisions) = jax.lax.scan(
        step, carry_init, input_seq
    )

    stage_cost = jnp.sum(costs)

    # Minimize time to hit puck
    positive_tf_cost = jnp.where(tf > 0.0, 0, 1e10)  # keep tf positive

    # Keep paths from becoming too short -> avoid jiggly behavior near puck
    enforce_last_step_collision = dt > dt_sim
    tf_cost = 1e2 * tf
    last_step_collision_cost = jnp.where(
        jnp.all(puck_collisions[:-1] == 0) & (puck_collisions[-1] == 1), 0, 1e10
    )
    any_step_collision_cost = jnp.where(jnp.any(puck_collisions == 1), 0, 1e10)
    enforce_collision_cost = jnp.where(
        enforce_last_step_collision,
        tf_cost + last_step_collision_cost,
        any_step_collision_cost,
    )

    # Puck going to goal
    (
        post_traj,
        will_r2p_collision,
        will_score,
        hit_y,
        wall_bounce_sequence,
        stopped_at,
    ) = rollout_controller(
        x_final,
        physics,
        H_goal_check,
        dt_sim,
        def_controller,
        r2_controller,
    )

    goal_cost = jnp.where(will_score == 1, 0, 1e10)  # 0: no goal, 1: goal left, 2: goal right
    enemy_robot_collision_cost = jnp.where(will_r2p_collision == 0, 0, 1e10)  # punish hiting enemy robot
    

    puck_pos_post = post_traj[:, 12:14]
    r1_pos_post = post_traj[:, :2]
    dist_post = jnp.linalg.norm(puck_pos_post - r1_pos_post, axis=1)
    valid_mask = jnp.arange(H_goal_check) < stopped_at
    masked_dist_post = jnp.where(valid_mask, dist_post, 0.0)
    valid_count = jnp.sum(valid_mask)
    avg_dist_post = jnp.where(
        valid_count > 0,
        jnp.sum(masked_dist_post) / valid_count,
        0.0,
    )
    proximity_reward = -1e1 * avg_dist_post

    total_cost = (
        stage_cost
        + positive_tf_cost
        + enforce_collision_cost
        + goal_cost
        + enemy_robot_collision_cost
        + proximity_reward
    )


    hit_wall = hit_y != -1.0
    should_print = (sample_idx % 100 == 0) & hit_wall

    # jax.lax.cond(
    #     should_print,
    #     lambda _: print_cost_overview(
    #         stage_cost,
    #         positive_tf_cost,
    #         enforce_collision_cost,
    #         goal_cost,
    #         enemy_robot_collision_cost,
    #         proximity_reward,
    #         total_cost,
    #         hit_y,
    #         will_score,
    #         will_r2p_collision,
    #         stopped_at,
    #     ),
    #     lambda _: 0,
    #     operand=None,
    # )

    return trajectory, total_cost, wall_bounce_sequence
