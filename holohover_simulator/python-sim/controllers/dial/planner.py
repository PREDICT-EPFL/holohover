import jax
import jax.numpy as jnp

from .rollout import rollout, rollout_dial, rollout_controller


def build_dial_path_planner(config, def_controller, imagined_r2_controller):

    d = config["dial"]
    H = d["H"]
    Nw = d["Nw"]
    temperature = d["update_temperature"]
    u_max = jnp.array(config["simulation"]["u_limits"])
    n_debug = d["n_debug_trajectories"]

    dt_sim = 1 / config["simulation"]["hz"]
    H_goal_check = int(round(d["goal_check_horizon"] / dt_sim))

    # Number of diffusion steps
    N = d["N"]

    beta1 = d["beta1"]
    beta2 = d["beta2"]

    R = jnp.diag(jnp.array(d["R_diag"]))

    pp_scale = jnp.diag(jnp.array(d["path_planning_scale"]))
    tf_scale = d["tf_scale"]

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
            candidate_inputs = input_sequences + input_mean  # not clipped: the simulator clips

            tf_sequence = W[:, -1]  # shape (Nw, 1)
            candidate_tf = tf_sequence + tf_mean

            batched_rollout = jax.vmap(
                rollout_dial,
                in_axes=(None, 0, 0, None, None, None, None, None, None, None),
            )

            trajs, raw_costs, _ = batched_rollout(
                x0,
                candidate_inputs,
                candidate_tf,
                R,
                physics,
                H,
                H_goal_check,
                dt_sim,
                def_controller,
                imagined_r2_controller,
            )

            shifted_costs = raw_costs - jnp.min(raw_costs)
            weights = jnp.exp(-shifted_costs / temperature)
            weights /= jnp.sum(weights)

            input_new = jnp.sum(
                weights[:, None, None] * (candidate_inputs), axis=0
            )

            tf_new = jnp.sum(weights * (candidate_tf))

            # Return a couple of example trajectories for visuals and debugging
            # show both xy coordinate of robot and cost of each trajectory
            # traj has shape (Nw, H, 18)
            example_trajs = trajs[:n_debug, :, :2] # 5 examples, only xy of robot
            example_costs = raw_costs[:n_debug]

            return (key, input_new, tf_new), (example_trajs, example_costs)

        input_prev = plan_prev[:, 6:]  # (x,y,phi,vx,vy,omega,Fx,Fy,T)
        (key, input_final, tf_final), (diffusion_trajs, diffusion_costs) = jax.lax.scan(
            diffusion_step,
            (key, input_prev, tf_prev),
            jnp.arange(N) # perform N (e.g. 10) diffusion steps
        )

        input_final = jnp.clip(input_final, -u_max, u_max) # prevent exploding
        tf_final = jnp.clip(tf_final, 1e-3, 5.0) # prevent exploding or negative time

        # rollout again, because of collisions etc. trajectories can't simply be weighted like the inputs
        # This is done for animation purposes, to show the trajectory of the final planned path
        trajectory, _ = rollout(
            x0,
            input_final,
            physics,
            tf_final / H,
            imagined_r2_controller,
        )
        r1_trajectory = trajectory[:, :6]

        # Generate future trajectory prediction (from collision point)
        # This is purely for animation purposes as well
        final_state = trajectory[-1, :]
        future_trajectory_post, _, _, _, _, _ = rollout_controller(
            final_state,
            physics,
            H_goal_check,
            dt_sim,
            def_controller, # Robot1 controller after puck collision
            imagined_r2_controller, # What Robot1 thinks Robot2 will do
        )
        # Include entire imagined sequence for animation:
        # 1) pre-collision planned rollout, 2) post-collision controller rollout.
        future_trajectory = jnp.concatenate((trajectory, future_trajectory_post), axis=0)
        path_plan = jnp.hstack((r1_trajectory, input_final))
        return key, path_plan, tf_final, future_trajectory, diffusion_trajs, diffusion_costs

    return jax.jit(path_planner)
