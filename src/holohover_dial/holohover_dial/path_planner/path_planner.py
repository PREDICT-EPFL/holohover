import jax
import jax.numpy as jnp
import interpax

from rollout import rollout, rollout_dial, rollout_controller


wall_bounce_patterns = jnp.array(
    [
        0,  # no bounces
        1,
        2,
        3,
        4,
        5,
        6,  # single bounce
        11,
        21,
        31,
        41,
        51,
        61,  # double bounces, first bounce on wall 1, second bounce on wall 1-6
        12,
        22,
        32,
        42,
        52,
        62,
        13,
        23,
        33,
        43,
        53,
        63,
        14,
        24,
        34,
        44,
        54,
        64,
        15,
        25,
        35,
        45,
        55,
        65,
        16,
        26,
        36,
        46,
        56,
        66,
    ]
).sort()


def build_dial_path_planner(robot_id, def_controller, imagined_enemy_controller):

    H = 15
    K = 6
    dt_pp = 0.5
    Nw = 2000
    temperature = 1e1
    min_pattern_samples = 20

    goal_check_horizon = 1.0
    dt_sim = 0.01
    H_goal_check = jnp.round(goal_check_horizon / dt_sim).astype(int)

    # Number of diffusion steps
    N = 8

    beta1 = 0.25
    beta2 = 1

    u_max = jnp.array([0.35, 0.35, 0.005])

    pp_scale = jnp.diag(jnp.array([0.01, 0.01, 0.5]))
    tf_scale = 0.01

    knot_times = jnp.linspace(0.0, 1.0, K + 1, endpoint=False)
    control_times = jnp.linspace(0.0, 1.0, H, endpoint=False)

    def knots_to_control(knots, initial_control, initial_rate):
        spline_bc = ((1, initial_rate), "not-a-knot")
        knots_complete = jnp.vstack((initial_control, knots.reshape(K, 3)))
        spline = interpax.CubicSpline(
            knot_times, knots_complete, axis=0, bc_type=spline_bc, check=False
        )
        return spline(control_times)

    # Build base covariance blocks (time decay)
    base_blocks = []
    for k in range(K):  # Covariance increases with knot time
        exponent = -(K - k) / (beta2 * K)
        block = pp_scale * jnp.exp(exponent)
        base_blocks.append(block)
    base_blocks.append(jnp.array([tf_scale]))  # For end time tf decision variable
    base_cov = jax.scipy.linalg.block_diag(*base_blocks)

    # Precompute diffusion covariance schedule
    dial_covariances = []
    for i in range(N):
        shrink = jnp.exp(-i / (beta1 * N))
        dial_covariances.append(base_cov * shrink)
    dial_covariances = jnp.stack(dial_covariances)
    # print(jnp.sqrt(dial_covariances[0, :3, :3]))  # 68% confidence interval
    # print(jnp.sqrt(dial_covariances[0, 12:, 12:]))
    # print(jnp.sqrt(dial_covariances[-1, :3, :3]))
    # print(jnp.sqrt(dial_covariances[-1, 12:, 12:]))

    def path_planner(key, x0, spline_prev, tf_prev, cost_prev, lock_tf):
        knot_prev = spline_prev(knot_times)[1:]
        initial_knot = spline_prev(0.0)
        initial_rate = spline_prev.derivative()(0.0)

        def sample_candidates(key, knot_mean, tf_mean, cov_idx):
            key, subkey = jax.random.split(key)

            cov_i = dial_covariances[cov_idx]
            R_i = temperature * jnp.linalg.inv(cov_i[: 3 * K, : 3 * K])

            W = jax.random.multivariate_normal(
                subkey,
                mean=jnp.zeros(3 * K + 1),  # +1 for tf
                cov=cov_i,
                shape=(Nw,),  # generate Nw (e.g. 1000) samples in parallel
            )

            tf_sequence = W[:, -1]  # shape (Nw, 1)
            tf_sequence = jnp.where(lock_tf, jnp.zeros_like(tf_sequence), tf_sequence)
            candidate_tf = tf_sequence + tf_mean
            candidate_tf = jnp.clip(
                candidate_tf, dt_pp, 5.0
            )  # prevent exploding or negative time

            knot_sequences = W[:, :-1].reshape(Nw, K, 3)  # exclude tf here
            candidate_knots = knot_sequences + knot_mean
            candidate_knots = jnp.clip(candidate_knots, -u_max, u_max)
            initial_rate_scaled = initial_rate * candidate_tf[:, None] / tf_prev
            candidate_inputs = jax.vmap(knots_to_control, in_axes=(0, None, 0))(
                candidate_knots, initial_knot, initial_rate_scaled
            )

            return key, candidate_knots, candidate_inputs, candidate_tf, R_i

        def compute_normalized_weights(mask, shifted_costs):
            unnormalized = jnp.where(mask, jnp.exp(-shifted_costs / temperature), 0.0)
            denom = jnp.sum(unnormalized)
            return jnp.where(
                denom > 1e-12,
                unnormalized / denom,
                jnp.ones_like(unnormalized) / unnormalized.shape[0],
            )

        def diffusion_step(carry, i):
            key, knot_mean, tf_mean, _ = carry
            key, candidate_knots, candidate_inputs, candidate_tf, R_i = (
                sample_candidates(key, knot_mean, tf_mean, i)
            )

            batched_rollout = jax.vmap(
                rollout_dial,
                in_axes=(None, 0, 0, 0, 0, None, None, None, None, None, None, None, None),
            )

            trajs, raw_costs, wall_bounce_seq = batched_rollout(
                x0,
                candidate_inputs,
                candidate_knots,
                jnp.arange(Nw),
                candidate_tf,
                R_i,
                physics,
                H,
                H_goal_check,
                dt_sim,
                robot_id,
                def_controller,
                imagined_enemy_controller,
            )

            shifted_costs = raw_costs - jnp.min(raw_costs)

            # Count rejected samples (wall bounce patter =67)
            n_rejected = jnp.sum(wall_bounce_seq == 67)
            # jax.debug.print(
            #     "Robot {robot_id} Diffusion step {i}: Rejected {n_rejected} out of {Nw} samples due to wall bounces",
            #     robot_id=robot_id,
            #     i=i,
            #     n_rejected=n_rejected,
            #     Nw=Nw,
            # )

            # Print cost overview
            # jax.debug.print("Diffusion step {i}: cost range [{c_min}, {c_max}], mean {c_mean}",
            #                 i=i, c_min=jnp.min(raw_costs), c_max=jnp.max(raw_costs), c_mean=jnp.mean(raw_costs))

            # Complex approach: Paper does this with DBSCAN clustering, we can do it with patterns
            pattern_idx = jnp.searchsorted(wall_bounce_patterns, wall_bounce_seq)

            group_cost_sum = jnp.bincount(
                pattern_idx,
                weights=shifted_costs,
                length=wall_bounce_patterns.shape[0],
            )
            group_size = jnp.bincount(
                pattern_idx,
                length=wall_bounce_patterns.shape[0],
            )
            group_cost = group_cost_sum / jnp.maximum(group_size, 1.0)

            valid_groups = group_size >= min_pattern_samples
            has_valid_group = jnp.any(valid_groups)
            best_pattern_idx = jnp.argmin(jnp.where(valid_groups, group_cost, jnp.inf))
            best_pattern_id = wall_bounce_patterns[best_pattern_idx]
            mask_best_pattern = wall_bounce_seq == best_pattern_id

            # Fallback: if no pattern has enough samples, use all candidates.
            selected_mask = jnp.where(
                has_valid_group,
                mask_best_pattern,
                jnp.ones_like(mask_best_pattern, dtype=bool),
            )
            weights = compute_normalized_weights(selected_mask, shifted_costs)

            # Simpler approach: just use the one very best candidate
            # best_idx = jnp.argmin(shifted_costs)
            # weights = jnp.zeros_like(shifted_costs)
            # weights = weights.at[best_idx].set(1.0)
            # best_pattern_id = wall_bounce_seq[best_idx]
            # cost = raw_costs[best_idx]

            # Standard approach: use all candidates, weighted by their cost
            # weights = compute_normalized_weights(jnp.ones_like(shifted_costs, dtype=bool), shifted_costs)

            # Alternative approach: Use only candidates from the biggest group
            # best_pattern_idx = jnp.argmax(group_size)
            # mask_biggest_group = pattern_idx == best_pattern_idx
            # weights = compute_normalized_weights(mask_biggest_group, shifted_costs)

            best_pattern_id = wall_bounce_patterns[best_pattern_idx]
            cost = group_cost[best_pattern_idx]
            knot_new = jnp.sum(weights[:, None, None] * candidate_knots, axis=0)
            tf_new = jnp.sum(weights * candidate_tf)

            # Debug print
            # jax.debug.print("Diffusion step {i}: best pattern {pattern} with cost {cost}, tf={tf}",
            #                 i=i, pattern=best_pattern_id, cost=cost, tf=tf_new)
            # jax.debug.print("Group sizes: {sizes}", sizes=group_size[:7])


            return (key, knot_new, tf_new, cost), None

        (key, knot_final, tf_final, cost_final), _ = jax.lax.scan(
            diffusion_step,
            (key, knot_prev, tf_prev, cost_prev),
            jnp.arange(N),
        )

        spline_final = interpax.CubicSpline(
            knot_times, jnp.vstack((initial_knot, knot_final)), axis=0, check=False
        )

        # jax.debug.print("Previous cost: {cost_prev}, new cost: {cost_after_rollout}, final cost: {cost_final}",
        #                 cost_prev=cost_prev, cost_after_rollout=cost_after_rollout, cost_final=cost_final)
        # jax.debug.print("After rollout diffusion: cost={cost_final}, tf={tf_final}",
        #                 cost_final=cost_final, tf_final=tf_final)

        # rollout again for animation, trajectories can't simply be weighted like the inputs
        input_final = spline_final(control_times)
        input_final = jnp.clip(input_final, -u_max, u_max)
        trajectory, r1_collisions_detected, r2_collisions_detected = rollout(
            x0,
            input_final,
            physics,
            tf_final / H,
            robot_id,
            imagined_enemy_controller,
        )
        my_trajectory = jnp.where(robot_id==1, trajectory[:, :6], trajectory[:, 6:12])

        # Generate future trajectory prediction for animation
        final_state = trajectory[-1, :]
        future_trajectory_post, will_r1p_collision, will_r2p_collision, will_score, _, _, _ = (
            rollout_controller(
                final_state,
                physics,
                H_goal_check,
                dt_sim,
                robot_id,
                def_controller,  # Robot controller after puck collision
                imagined_enemy_controller,  # What Robot thinks the enemy will do
            )
        )
        # Include entire imagined sequence for animation:
        future_trajectory = jnp.concatenate(
            (trajectory, future_trajectory_post), axis=0
        )
        path_plan = jnp.hstack((my_trajectory, input_final))

        # Signal whether this plan predicts an Robot-puck hit in my own half.
        has_collision = jnp.where(robot_id==1,
                                  jnp.any(r1_collisions_detected) | will_r1p_collision,
                                  jnp.any(r2_collisions_detected) | will_r2p_collision)
        avoid_enemy = jnp.where(robot_id==1,
                                ~jnp.any(r2_collisions_detected) & ~will_r2p_collision,
                                ~jnp.any(r1_collisions_detected) & ~will_r1p_collision)
        robot_x_at_hit = my_trajectory[-1, 0]
        puckx_at_hit = trajectory[-1, 12]
        hit_in_own_half = jnp.where(robot_id==1,
                                    (robot_x_at_hit > 0.5 * physics.table.width) & (puckx_at_hit > 0.5 * physics.table.width),
                                    (robot_x_at_hit < 0.5 * physics.table.width) & (puckx_at_hit < 0.5 * physics.table.width))
        correct_goal = will_score == robot_id # will_score=1 -> goal left side

        # debug print for all the conditions:
        # jax.debug.print("Robot {robot_id} collision: {collision}, avoid_enemy: {avoid_enemy}, hit_in_own_half: {hit_in_own_half}, correct_goal: {correct_goal}",
        #                 robot_id=robot_id, collision=has_collision, avoid_enemy=avoid_enemy, hit_in_own_half=hit_in_own_half, correct_goal=correct_goal)

        score_from_own_half = correct_goal & has_collision & avoid_enemy & hit_in_own_half

        return (
            key,
            spline_final,
            path_plan,
            tf_final,
            cost_final,
            future_trajectory,
            None,
            None,
            score_from_own_half,
        )

    return jax.jit(path_planner)
