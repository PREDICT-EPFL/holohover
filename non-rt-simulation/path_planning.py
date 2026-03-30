import jax
import jax.numpy as jnp

from rollout import rollout, rollout_dial, rollout_controller


def build_dial_path_planner(config, def_controller):

    H = config["simulation"]["H"]
    Nw = config["dial"]["Nw"]
    temperature = config["dial"]["update_temperature"]
    u_max = jnp.array(config["simulation"]["u_limits"])
    n_debug = config["simulation"]["n_debug_trajectories"]

    goal_check_horizon = config["simulation"]["goal_check_horizon"]
    dt_sim = 1 / config["simulation"]["hz"]
    H_goal_check = jnp.round(goal_check_horizon / dt_sim).astype(int)

    # Number of diffusion steps
    N = config["dial"]["N"]

    beta1 = config["dial"]["beta1"]
    beta2 = config["dial"]["beta2"]

    Q = jnp.diag(jnp.array(config["dial"]["Q_diag"]))
    R = jnp.diag(jnp.array(config["dial"]["R_diag"]))

    pp_scale = jnp.diag(jnp.array(config["dial"]["path_planning_scale"]))
    tf_scale = config["dial"]["tf_scale"]

    # Build base covariance blocks (time decay)
    base_blocks = []
    for h in range(H):  # Covariance increases with time
        exponent = -(H - h) / (beta2 * H)
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
    # print(dial_covariances[0, :3, :3])
    # print(dial_covariances[0, 42:, 42:])
    # print(dial_covariances[-1, :3, :3])
    # print(dial_covariances[-1, 42:, 42:])

    def path_planner(key, x0, plan_prev, tf_prev, physics):

        def diffusion_step(carry, i):
            key, input_mean, tf_mean = carry
            key, subkey = jax.random.split(key)

            cov_i = dial_covariances[i]

            W = jax.random.multivariate_normal(
                subkey,
                mean=jnp.zeros(3 * H + 1), # +1 for tf
                cov=cov_i,
                shape=(Nw,) # generate Nw (e.g. 1000) samples in parallel
            )

            input_sequences = W[:, :-1].reshape(Nw, H, 3)  # exclude tf here

            tf_sequence = W[:, -1]  # shape (Nw, 1)

            batched_rollout = jax.vmap(
                rollout_dial,
                in_axes=(None, 0, 0, None, None, None, None, None, None, None),
            )

            trajs, costs, _ = batched_rollout(
                x0,
                input_sequences + input_mean,
                tf_sequence + tf_mean,
                Q,
                R,
                physics,
                H,
                def_controller,
                H_goal_check,
                dt_sim,
            )

            costs -= jnp.min(costs)
            weights = jnp.exp(-costs/temperature)
            weights /= jnp.sum(weights)

            input_new = jnp.sum(
                weights[:, None, None] * (input_sequences + input_mean), axis=0
            )

            tf_new = jnp.sum(weights * (tf_sequence + tf_mean))


            # Return a couple of example trajectories for visuals and debugging
            # show both xy coordinate of robot and cost of each trajectory
            # traj has shape (Nw, H, 12)
            example_trajs = trajs[:n_debug, :, :2] # 5 examples, only xy of robot
            example_costs = costs[:n_debug]

            return (key, input_new, tf_new), (example_trajs, example_costs)

        input_prev = plan_prev[:, 6:]  # (x,y,phi,vx,vy,omega,Fx,Fy,T)
        (key, input_final, tf_final), (diffusion_trajs, diffusion_costs) = jax.lax.scan(
            diffusion_step,
            (key, input_prev, tf_prev),
            jnp.arange(N) # perform N (e.g. 10) diffusion steps
        )

        input_final = jnp.clip(input_final, -u_max, u_max) # prevent exploding

        # rollout again, because of collisions etc. trajectories can't simply be weighted like the inputs
        trajectory, collisions_detected = rollout(
            x0, input_final, physics, tf_final / H
        )
        robot_trajectory = trajectory[:, :6]

        # Generate future trajectory prediction (from collision point)
        # This is purely for animation purposes
        idx_collision = jnp.argmax(collisions_detected)  # detect FIRST RP-collision
        collision_state = trajectory[idx_collision, :]
        # final_state = trajectory[-1, :]
        future_trajectory, _, _, _ = rollout_controller(
            collision_state, def_controller, physics, H_goal_check, dt_sim
        )
        future_trajectory = jnp.concatenate((collision_state[None, :], future_trajectory))
        path_plan = jnp.hstack((robot_trajectory, input_final))
        return key, path_plan, tf_final, future_trajectory, diffusion_trajs, diffusion_costs

    return jax.jit(path_planner)
