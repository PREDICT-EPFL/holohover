"""
DIAL path planner (Till's non-rt-simulation/path_planning.py, report Section 3.2) for robot 1.

The decision variables are dial.n_knots spline knots of the input (the input at time 0 is kept
from the previous plan) and the time to impact tf; the H inputs of a plan are the spline sampled
at k tf / H. Each of the dial.N diffusion steps samples dial.Nw candidates around the current
mean with a shrinking covariance, rolls them out (rollout.rollout_dial), groups them by the wall
bounce sequence of their shot, and moves the mean to the cost-weighted average of the best group.
"""

import jax
import jax.numpy as jnp
import interpax

from .rollout import rollout, rollout_controller, rollout_dial

# Wall bounce sequences a shot can have: none, one bounce on wall w (1-6), or two (w2 w1 as digits)
WALL_BOUNCE_PATTERNS = jnp.array(
    [0, 1, 2, 3, 4, 5, 6] + [10 * w2 + w1 for w1 in range(1, 7) for w2 in range(1, 7)]).sort()
MIN_PATTERN_SAMPLES = 20  # a bounce sequence with fewer samples is not selected


def build_dial_path_planner(config, def_controller, enemy_controller):
    """
    Returns planner(key, x0, spline_prev, tf_prev, cost_prev, physics) ->
    (key, spline, path_plan, tf, cost, future, diffusion_trajs, diffusion_costs, valid_shot):
    spline: the input spline over normalised time [0, 1]; path_plan (H, 9): planned robot 1 states
    and inputs; future: the planned states then the shot until the puck reaches the end wall;
    diffusion_trajs/costs: robot 1 paths and costs of the first dial.n_debug_trajectories samples
    of each diffusion step; valid_shot: the plan hits the puck in our half and scores without
    robot 2 touching the puck.
    """
    d = config["dial"]
    H, K, N, Nw = d["H"], d["n_knots"], d["N"], d["Nw"]
    temperature = d["update_temperature"]
    n_debug = d["n_debug_trajectories"]
    dt_goal = 1 / d["goal_check_hz"]
    H_goal_check = int(round(d["goal_check_horizon"] / dt_goal))
    u_max = jnp.array(config["simulation"]["u_limits"])

    knot_times = jnp.linspace(0.0, 1.0, K + 1, endpoint=False)
    control_times = jnp.linspace(0.0, 1.0, H, endpoint=False)

    # Sampling covariance: grows with the knot time, then shrinks over the diffusion steps
    pp_scale = jnp.diag(jnp.array(d["path_planning_scale"]))
    blocks = [pp_scale * jnp.exp(-(K - k) / (d["beta2"] * K)) for k in range(K)] + [jnp.array([d["tf_scale"]])]
    base_cov = jax.scipy.linalg.block_diag(*blocks)
    covariances = jnp.stack([base_cov * jnp.exp(-i / (d["beta1"] * N)) for i in range(N)])

    def knots_to_control(knots, initial_control, initial_rate):
        spline = interpax.CubicSpline(knot_times, jnp.vstack((initial_control, knots.reshape(K, 3))), axis=0,
                                      bc_type=((1, initial_rate), "not-a-knot"), check=False)
        return spline(control_times)

    def path_planner(key, x0, spline_prev, tf_prev, cost_prev, physics):
        initial_knot = spline_prev(0.0)
        initial_rate = spline_prev.derivative()(0.0)

        def diffusion_step(carry, i):
            key, knot_mean, tf_mean, _ = carry
            key, subkey = jax.random.split(key)
            W = jax.random.multivariate_normal(subkey, mean=jnp.zeros(3 * K + 1), cov=covariances[i], shape=(Nw,))

            candidate_tf = jnp.clip(W[:, -1] + tf_mean, 0, 5.0)
            candidate_knots = jnp.clip(W[:, :-1].reshape(Nw, K, 3) + knot_mean, -u_max, u_max)
            candidate_inputs = jax.vmap(knots_to_control, in_axes=(0, None, 0))(
                candidate_knots, initial_knot, initial_rate * candidate_tf[:, None] / tf_prev)

            trajs, raw_costs, bounce_seq = jax.vmap(
                rollout_dial, in_axes=(None, 0, 0, 0, None, None, None, None, None, None))(
                x0, candidate_inputs, candidate_knots, candidate_tf, physics, H, H_goal_check, dt_goal,
                def_controller, enemy_controller)
            shifted = raw_costs - jnp.min(raw_costs)

            # Sample clustering by bounce sequence: average the samples of the cheapest sequence only
            n_patterns = WALL_BOUNCE_PATTERNS.shape[0]
            group = jnp.searchsorted(WALL_BOUNCE_PATTERNS, bounce_seq)  # invalid shots (67) fall off the end
            group_size = jnp.bincount(group, length=n_patterns)
            group_cost = jnp.bincount(group, weights=shifted, length=n_patterns) / jnp.maximum(group_size, 1.0)
            valid_groups = group_size >= MIN_PATTERN_SAMPLES
            best = jnp.argmin(jnp.where(valid_groups, group_cost, jnp.inf))
            selected = jnp.where(jnp.any(valid_groups), bounce_seq == WALL_BOUNCE_PATTERNS[best], True)

            weights = jnp.where(selected, jnp.exp(-shifted / temperature), 0.0)
            total = jnp.sum(weights)
            weights = jnp.where(total > 1e-12, weights / total, 1.0 / Nw)

            knot_new = jnp.sum(weights[:, None, None] * candidate_knots, axis=0)
            tf_new = jnp.sum(weights * candidate_tf)
            return (key, knot_new, tf_new, group_cost[best]), (trajs[:n_debug, :, :2], raw_costs[:n_debug])

        (key, knots, tf, cost), (diffusion_trajs, diffusion_costs) = jax.lax.scan(
            diffusion_step, (key, spline_prev(knot_times)[1:], tf_prev, cost_prev), jnp.arange(N))
        spline = interpax.CubicSpline(knot_times, jnp.vstack((initial_knot, knots)), axis=0, check=False)

        # Roll the averaged plan out again: the states of a plan cannot be averaged like its inputs
        inputs = jnp.clip(spline(control_times), -u_max, u_max)
        trajectory, r1_cols, r2_cols = rollout(x0, inputs, physics, tf / H, enemy_controller)
        post, will_r1_col, will_r2_col, will_score, _, _, _ = rollout_controller(
            trajectory[-1], physics, H_goal_check, dt_goal, def_controller, enemy_controller)
        future = jnp.concatenate((trajectory, post), axis=0)
        path_plan = jnp.hstack((trajectory[:, :6], inputs))

        half = 0.5 * physics.table.width
        valid_shot = ((will_score == 1)  # left goal
                      & (jnp.any(r1_cols == 1) | (will_r1_col == 1))
                      & ~jnp.any(r2_cols == 1) & (will_r2_col == 0)
                      & (trajectory[-1, 0] > half) & (trajectory[-1, 12] > half))
        return key, spline, path_plan, tf, cost, future, diffusion_trajs, diffusion_costs, valid_shot

    return jax.jit(path_planner)
