import os
from datetime import datetime
import json
import pickle

import jax
import jax.numpy as jnp

jax.config.update("jax_disable_jit", False)

from dynamics import Table, Entity, PhysicsParams, physics_step
from controller import (
    build_pd_controller,
    build_mpc_controller,
    build_interpolation_controller,
)
from path_planning import (
    build_dial_path_planner,
)
from rollout import rollout, rollout_pd


def run_sim(config=None):
    if not config:
        print("No config file specified...")
        return

    H = config["simulation"]["H"]
    N = config["dial"]["N"]
    dt = 1 / config["simulation"]["hz"]
    steps = int(config["simulation"]["duration"] / dt)
    dt_pp = 1 / config["simulation"]["path_planning_hz"]
    u_max = jnp.array(config["simulation"]["u_limits"])
    n_debug = config["simulation"]["n_debug_trajectories"]

    goal_check_horizon = config["simulation"]["goal_check_horizon"]
    dt_future_traj = 1 / config["simulation"]["hz"]
    H_future_traj = jnp.round(goal_check_horizon / dt_future_traj).astype(int)

    table = Table(**config["table"])

    robot_params = Entity(
        radius=config["robot"]["radius"],
        mass=config["robot"]["mass"],
        inertia=0.5 * config["robot"]["mass"] * config["robot"]["radius"] ** 2,
        u_limits=jnp.array(config["simulation"]["u_limits"]),
    )

    puck_params = Entity(
        radius=config["puck"]["radius"],
        mass=config["puck"]["mass"],
        inertia=0.5 * config["puck"]["mass"] * config["puck"]["radius"] ** 2,
        u_limits=jnp.array([0, 0, 0]),
    )

    physics_params = PhysicsParams(table=table, robot=robot_params, puck=puck_params)
    a_max = robot_params.u_limits[0] / robot_params.mass

    att_controller = build_interpolation_controller(config)
    def_controller = build_pd_controller(config, physics_params)
    path_planner = build_dial_path_planner(config, def_controller)
    # return
    replan_at = jnp.round(dt_pp / dt).astype(int)

    def sim_step(carry, _):
        (key, mode, state, plan, tf, future_traj,time_step, score,
         diffusion_trajs, diffusion_costs) = carry
        # jax.debug.print("t={:.2f}, mode={}", time_step * dt, mode)

        build_new_path = time_step % replan_at == 0

        ### ATTACK MODE ###
        def attack_branch(_):
            def do_plan(_):
                dt_plan = tf / H
                roll_by = jnp.round(dt_pp / dt_plan).astype(int)

                rolled_indices = jnp.arange(H) + roll_by
                roll_plan = plan[jnp.minimum(rolled_indices, H - 1)]
                # roll_plan = jnp.where((rolled_indices < H)[:, None], roll_plan, 0.0)
                # roll_plan = plan

                (new_key, new_plan, new_tf, new_future_traj,
                 new_diffusion_trajs, new_diffusion_costs) = path_planner(
                    key, state,
                    roll_plan,
                    tf,
                    physics_params,
                )
                jax.debug.print("tf: {}", tf)
                # jax.debug.print("Fy {}", new_plan[:, 7])
                # jax.debug.print("will_score {}", will_score)
                return new_key, new_plan, new_tf, new_future_traj, new_diffusion_trajs, new_diffusion_costs

            def skip_plan(_):
                return key, plan, tf, future_traj, diffusion_trajs, diffusion_costs

            (new_key, new_plan, new_tf, new_future_traj,
             new_diffusion_trajs, new_diffusion_costs) = jax.lax.cond(
                build_new_path, do_plan, skip_plan, operand=None
            )

            # 2. Control -> follow pre-planned trajectory (low-level, high frequency)
            time_pp = (time_step % replan_at) * dt  # how far down the path plan we are
            u0 = att_controller(state, new_plan, tf, time_pp)

            # jax.debug.print("\nstep {}\n time_pp {}\n u0 {}\n plan {}\n state {}\n tf {}", time_step, time_pp, jnp.round(u0, 4), plan[:, 6:], state, tf)
            return new_key, new_plan, new_tf, new_future_traj, u0, new_diffusion_trajs, new_diffusion_costs

        ### DEFENSE MODE ###
        def defense_branch(_):
            u0 = def_controller(state)
            return key, plan, tf, future_traj, u0, diffusion_trajs, diffusion_costs

        # Calculate new action
        (new_key, new_plan, new_tf, new_future_traj, u0,
         new_diffusion_trajs, new_diffusion_costs) = jax.lax.cond(
            mode == 1, attack_branch, defense_branch, operand=None
        )
        u0 = jnp.clip(u0, -u_max, u_max)

        # 3. Apply action, stop if either side scored
        # def scored_branch(_): # Do nothing
        #     return state, False, 0, jnp.zeros(6)

        # def normal_branch(_): # Regular physics step
        #     x_next, collision_detected_next, has_scored_next, _, _ = physics_step(state, u0, physics_params, dt)
        #     return x_next, collision_detected_next, has_scored_next, jnp.zeros(6)

        # state_next, collision_detected, has_scored, _ = jax.lax.cond(jnp.sum(score)==1, scored_branch, normal_branch, operand=None)
        state_next, collision_detected, has_scored, _, _ = physics_step(
            state, u0, physics_params, dt
        )

        # 4. Change mode and increment score
        # Collision happened: Attack -> Defense
        mode_next = jnp.where(collision_detected & (mode == 1), 0, mode)
        # Puck unreachable for opponent: Defense -> Attack (simplified for now)
        opponent_x = 0.2
        opponent_vx = 1.0
        puck_x = state_next[6]
        puck_vx = state_next[9]

        delta_v = opponent_vx - puck_vx
        r = jnp.maximum(puck_x - opponent_x, 0)
        time_to_collision = -delta_v / a_max + jnp.sqrt(
            (delta_v / a_max) ** 2 + 2 * r / a_max
        )
        d = 0.5 * table.width - puck_x
        time_to_out_of_play = d / puck_vx

        puck_unreachable = time_to_collision > time_to_out_of_play
        in_own_half = state_next[0] > 0.5 * table.width
        # jax.debug.print("puck vx {}", puck_vx)
        # jax.debug.print("mode {}", mode)
        puck_pos_vel = puck_vx > 0.1

        # Attack if opponent likely cant reach puck in time and we are in defense mode.
        # mode_next = jnp.where(in_own_half & (puck_vx > 0) & (mode == 0), 1, mode_next)

        score_update = jnp.array([has_scored == 1, has_scored == 2], dtype=jnp.int32)
        new_score = score + score_update

        # 5. Roll time
        time_step = time_step + 1

        return ( # carry
            new_key,
            mode_next,
            state_next,
            new_plan,
            new_tf,
            new_future_traj,
            time_step,
            new_score,
            new_diffusion_trajs,
            new_diffusion_costs,
        ), ( # return
            state_next,
            u0,
            new_plan,
            new_tf,
            new_future_traj,
            mode_next,
            new_score,
            new_diffusion_trajs,
            new_diffusion_costs,
        )

    # Initialize Simulation:
    # state = [robot(6), puck(6)]
    rpos = config["robot"]["initial_position"]
    rvel = config["robot"]["initial_velocity"]
    ppos = config["puck"]["initial_position"]
    pvel = config["puck"]["initial_velocity"]
    init_state = jnp.array(rpos + rvel + ppos + pvel)

    key = jax.random.PRNGKey(0)
    rp_dist = jnp.linalg.norm(init_state[:2] - init_state[6:8])
    delta_v = jnp.linalg.norm(init_state[3:5] - init_state[9:11])
    # a_max defined above
    init_tf = -delta_v / a_max + jnp.sqrt((delta_v / a_max) ** 2 + 2 * rp_dist / a_max)
    # s = 0.5 * a * t^2 + v * t --> t = sqrt(2s/a)

    # PD to position where puck will be at tf
    puck_init_state = jnp.array([0, 0, 0, 0, 0, 0] + ppos + pvel)
    p_traj, _ = rollout(puck_init_state, jnp.zeros((H, 3)), physics_params, init_tf / H)
    ppos_at_tf = p_traj[-1, 6:9]
    init_traj, init_inputs = rollout_pd(
        init_state, ppos_at_tf, H, physics_params, init_tf / H
    )

    init_inputs = jnp.clip(init_inputs, -u_max, u_max)
    init_path_plan = jnp.hstack((init_traj[:, :6], init_inputs))

    init_time = 0  # obviously time starts at zero
    init_mode = 1  # 1=attack, 0=defense
    init_score = jnp.array([0, 0])
    init_future_traj = jnp.zeros((H_future_traj+1, 12))

    init_diffusion_trajs = jnp.zeros((N, n_debug, H, 2)) # 10 diff steps, 5 trajs each, only xy of robot
    init_diffusion_costs = jnp.zeros((N, n_debug))
    init_diffusion_costs = init_diffusion_costs.at[:, 0].set(1.0)

    _, (states, input_sequence, path_plans, tfs, future_trajs, modes, scores,
        diffusion_trajs, diffusion_costs) = jax.lax.scan(
        sim_step,
        (key, init_mode, init_state, init_path_plan, init_tf, init_future_traj,
         init_time, init_score, init_diffusion_trajs, init_diffusion_costs),
        None,
        length=steps)

    # Add respective initial element
    states = jnp.concatenate((init_state.reshape((1, 12)), states))
    input_sequence = jnp.concatenate((init_inputs[0].reshape((1, 3)), input_sequence))
    path_plans = jnp.concatenate((init_path_plan[None, ...], path_plans))
    tfs = jnp.concatenate((jnp.array([init_tf]), tfs.flatten()))
    modes = jnp.concatenate((jnp.array([init_mode]), modes.flatten()))
    scores = jnp.concatenate((init_score.reshape((1, 2)), scores))
    future_trajs = jnp.concatenate((init_future_traj[None, ...], future_trajs))
    diffusion_trajs = jnp.concatenate(
        (init_diffusion_trajs[None, ...], diffusion_trajs)
    )
    diffusion_costs = jnp.concatenate(
        (init_diffusion_costs[None, ...], diffusion_costs)
    )

    print("Simulation finished.")

    # Pickle results
    os.makedirs("simulation_data", exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    file_name = f"simulation_data/{timestamp}.pkl"

    with open(file_name, "wb") as f:
        pickle.dump(
            (
                states,
                input_sequence,
                path_plans,
                tfs,
                future_trajs,
                modes,
                scores,
                diffusion_trajs,
                diffusion_costs,
                physics_params,
                config,
            ),
            f,
        )
    print(f"Saved to {file_name}")


if __name__ == "__main__":
    run_sim()
