import jax
import jax.numpy as jnp
import jaxopt
import numpy as np
import control
import interpax


def build_interpolation_controller(config):
    dt = 1 / config["simulation"]["hz"]

    def controller(x0, input_spline, tf, time_pp):
        t_normalized = jnp.clip(time_pp / (tf + 1e-8), 0.0, 1.0)
        u0 = input_spline(t_normalized)y
        return u0

    return jax.jit(controller)


def build_pd_controller(target_state, robot_id: int):
    target_pos = target_state[:3]
    target_vel = target_state[3:6]

    start_idx = 0 if robot_id == 1 else 6

    def controller(x0):
        pos = x0[start_idx : start_idx + 3]
        vel = x0[start_idx + 3 : start_idx + 6]
        Kp = 20.0
        Kd = 6.0
        pos_error = target_pos - pos
        vel_error = target_vel - vel

        u0 = Kp * pos_error + Kd * vel_error
        return u0

    return jax.jit(controller)


def build_lqr_tracking_controller(target_state, robot_id: int, config, physics):
    dt = 1 / config["simulation"]["hz"]
    mass = physics.robot.mass
    inertia = physics.robot.inertia

    target_pos = target_state[:3]
    target_vel = target_state[3:6]

    A = np.block([[np.zeros((3, 3)), np.eye(3)], [np.zeros((3, 6))]])
    B = np.block([[np.zeros((3, 3))], [np.diag([1 / mass, 1 / mass, 1 / inertia])]])

    nx, nu = B.shape
    sys_cont = control.ss(A, B, np.eye(nx), np.zeros((nx, nu)))
    sys_disc = control.c2d(sys_cont, dt)
    Ad = np.array(sys_disc.A)
    Bd = np.array(sys_disc.B)

    Q = np.diag([10.0, 10.0, 10.0, 0.5, 0.5, 0.5])  # State cost
    R = np.diag([0.01, 0.01, 0.05])  # Control cost

    K, _, _ = control.dlqr(Ad, Bd, Q, R)
    K = jnp.array(K)

    start_idx = 0 if robot_id == 1 else 6

    def controller(x0):
        pos = x0[start_idx : start_idx + 3]
        vel = x0[start_idx + 3 : start_idx + 6]
        pos_error = target_pos - pos
        vel_error = target_vel - vel
        error = jnp.concatenate([pos_error, vel_error])

        u0 = K @ error
        return u0

    return jax.jit(controller)


def build_puck_following_controller(robot_id: int):
    def controller(x0):
        puck_pos = x0[12:14]
        robot_pos = x0[:2] if robot_id == 1 else x0[6:8]

        pos_error = puck_pos - robot_pos
        Kp = 0.5
        u0_pos = Kp * pos_error

        u0 = jnp.array([u0_pos[0], u0_pos[1], 0.0])  # no rotation command
        return u0

    return jax.jit(controller)


def build_force_field_controller(home_state, table_width: float, robot_id: int):
    """Imagined opponent policy: home attraction + puck attraction + middle-line barrier."""
    home_pos = home_state[:2]
    mid_x = 0.5 * table_width

    side_sign = jnp.where(robot_id == 1, 1.0, -1.0)

    start_idx = 0 if robot_id == 1 else 6

    def controller(x0):
        robot_pos = x0[start_idx : start_idx + 2]
        robot_vel = x0[start_idx + 3 : start_idx + 5]
        puck_pos = x0[12:14]

        # Pull back to home while still reacting to puck position.
        k_home = 0.3
        k_puck = 0 * jnp.where(
            puck_pos[1] > -100.0, 0.6, 0.0
        )  # if puck oob (reset after goal) --> ignore
        k_damp = 0.1

        home_force = k_home * (home_pos - robot_pos)

        # Project puck target to own half so imagined opponent does not chase across center.
        margin = 0.05 * table_width
        puck_x_limited = jnp.where(
            robot_id == 1,
            jnp.minimum(puck_pos[0], mid_x - margin),
            jnp.maximum(puck_pos[0], mid_x + margin),
        )

        puck_target = jnp.array([puck_x_limited, puck_pos[1]])
        puck_force = k_puck * (puck_target - robot_pos)

        # Smooth repulsive barrier near the middle line.
        boundary_dist = side_sign * (robot_pos[0] - mid_x)
        barrier_strength = 12.0
        barrier = barrier_strength * jax.nn.softplus(-(boundary_dist - margin) / margin)
        barrier_force = jnp.array([side_sign * barrier, 0.0])

        damp_force = -k_damp * robot_vel

        u0_pos = home_force + puck_force + barrier_force + damp_force
        u0 = jnp.array([u0_pos[0], u0_pos[1], 0.0])  # no rotation command

        return u0

    return jax.jit(controller)


def build_do_nothing_controller():
    def controller(x0):
        return jnp.zeros(3)

    return jax.jit(controller)


def build_mpc_controller(config, physics, robot_id: int, final_only=False):
    H_mpc = config["simulation"]["H_mpc"]

    Q = jnp.diag(jnp.array(config["dial"]["Q_diag"]))
    R = jnp.diag(jnp.array(config["dial"]["R_diag"]))
    Q_terminal = 50.0 * Q
    nx = 6
    nu = 3
    start_idx = 0 if robot_id == 1 else 6

    solver = jaxopt.OSQP(tol=1e-3)

    x_min = physics.robot.radius
    x_max = physics.table.width - physics.robot.radius
    y_min = physics.robot.radius
    y_max = physics.table.height - physics.robot.radius
    wall_hit_margin = 2.0 * physics.robot.radius

    def wrap_angle(angle):
        return jnp.arctan2(jnp.sin(angle), jnp.cos(angle))

    def mirror_vertical(plan_states, wall_x):
        mirrored = plan_states.at[:, 0].set(2.0 * wall_x - plan_states[:, 0])
        mirrored = mirrored.at[:, 3].set(-plan_states[:, 3])
        # mirrored = mirrored.at[:, 2].set(wrap_angle(jnp.pi - plan_states[:, 2]))
        # mirrored = mirrored.at[:, 5].set(-plan_states[:, 5])
        return mirrored

    def mirror_horizontal(plan_states, wall_y):
        mirrored = plan_states.at[:, 1].set(2.0 * wall_y - plan_states[:, 1])
        mirrored = mirrored.at[:, 4].set(-plan_states[:, 4])
        # mirrored = mirrored.at[:, 2].set(wrap_angle(-plan_states[:, 2]))
        # mirrored = mirrored.at[:, 5].set(-plan_states[:, 5])
        return mirrored

    def unfold_first_wall_bounce(plan_states, tf, time_into_pp):
        positions = plan_states[:, :2]
        velocities = plan_states[:, 3:5]
        H_plan = plan_states.shape[0]
        idx = jnp.arange(H_plan - 1)
        inf = H_plan # value larger than any possible bounce step idx

        left_hit = (
            (velocities[:-1, 0] < 0.0)
            & (velocities[1:, 0] > 0.0)
            & ((positions[:-1, 0] <= x_min + wall_hit_margin) | (positions[1:, 0] <= x_min + wall_hit_margin))
        )
        right_hit = (
            (velocities[:-1, 0] > 0.0)
            & (velocities[1:, 0] < 0.0)
            & ((positions[:-1, 0] >= x_max - wall_hit_margin) | (positions[1:, 0] >= x_max - wall_hit_margin))
        )
        bottom_hit = (
            (velocities[:-1, 1] < 0.0)
            & (velocities[1:, 1] > 0.0)
            & ((positions[:-1, 1] <= y_min + wall_hit_margin) | (positions[1:, 1] <= y_min + wall_hit_margin))
        )
        top_hit = (
            (velocities[:-1, 1] > 0.0)
            & (velocities[1:, 1] < 0.0)
            & ((positions[:-1, 1] >= y_max - wall_hit_margin) | (positions[1:, 1] >= y_max - wall_hit_margin))
        )

        candidate_steps = jnp.array(
            [
                jnp.min(jnp.where(left_hit, idx, inf)),
                jnp.min(jnp.where(right_hit, idx, inf)),
                jnp.min(jnp.where(bottom_hit, idx, inf)),
                jnp.min(jnp.where(top_hit, idx, inf)),
            ]
        )
        wall_kind = jnp.argmin(candidate_steps)
        bounce_step = candidate_steps[wall_kind]
        has_bounce = bounce_step < inf
        
        # Check if bounce is in the future (not yet passed)
        bounce_time = bounce_step * tf / jnp.maximum(H_plan, 1)
        bounce_in_future = bounce_time > time_into_pp
        should_unfold = has_bounce & bounce_in_future

        mirrored_plan = jax.lax.switch(
            wall_kind,
            (
                lambda s: mirror_vertical(s, x_min),
                lambda s: mirror_vertical(s, x_max),
                lambda s: mirror_horizontal(s, y_min),
                lambda s: mirror_horizontal(s, y_max),
            ),
            plan_states,
        )
        suffix_mask = jnp.arange(H_plan) > bounce_step
        unfolded_plan = jnp.where(suffix_mask[:, None], mirrored_plan, plan_states)
        
        # Only apply unfolding if bounce is in the future
        return jnp.where(should_unfold, unfolded_plan, plan_states), should_unfold

    def interpolate_reference(plan_values, tf, sample_times):
        plan_times = jnp.arange(0.0, 1.0 + 1e-8, 1.0 / (plan_values.shape[0]))[1:]
        spline = interpax.CubicSpline(plan_times, plan_values, axis=0, check=False)
        normalized_sample_times = jnp.clip(
            sample_times / jnp.maximum(tf, 1e-8), 0.0, 1.0
        )
        return spline(normalized_sample_times)

    def build_qp(x0, x_ref, horizon):
        dt = horizon / H_mpc

        Ad, Bd = physics.system(dt)

        A_qp_left = jnp.kron(jnp.eye(H_mpc + 1), jnp.eye(nx))
        A_qp_left = A_qp_left + jnp.vstack(
            [
                jnp.zeros((nx, nx * (H_mpc + 1))),
                jnp.hstack(
                    [jnp.kron(jnp.eye(H_mpc), -Ad), jnp.zeros((H_mpc * nx, nx))]
                ),
            ]
        )
        A_qp_right = jnp.zeros((A_qp_left.shape[0], H_mpc * nu))
        A_qp_right = A_qp_right.at[nx:, :].set(
            A_qp_right[nx:, :] + jnp.kron(jnp.eye(H_mpc), -Bd)
        )
        A_qp = jnp.hstack([A_qp_left, A_qp_right])

        b_qp = jnp.concatenate([x0, jnp.zeros((H_mpc) * nx)])
        A_eq = A_qp
        b_eq = b_qp

        # Inequality constraints G*x <= h
        G = jnp.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0]])
        G_x = jnp.zeros((4 * H_mpc, nx * (H_mpc + 1)))
        G_u = jnp.kron(jnp.eye(H_mpc), G)
        G_qp = jnp.hstack([G_x, G_u])
        h = jnp.array([0.35, 0.35, 0.35, 0.35])

        h_qp = jnp.concatenate(
            [
                jnp.kron(jnp.ones((H_mpc, 1)), h).reshape(
                    -1,
                ),
            ]
        )

        # Cost function 1/2*x'*P*x
        P_qp = jnp.array(
            jax.scipy.linalg.block_diag(*([Q] * H_mpc + [Q_terminal] + [R] * H_mpc))
        )

        state_cost_linear = -jnp.concatenate(
            [
                jnp.reshape(x_ref[:-1] @ Q.T, (-1,)),
                Q_terminal @ x_ref[-1],
            ]
        )
        c_qp = jnp.concatenate(
            [
                state_cost_linear,
                jnp.zeros(H_mpc * nu)
            ]
        )

        return P_qp, c_qp, A_eq, b_eq, G_qp, h_qp

    def controller(x0, plan, tf, time_into_pp):
        plan_states = plan[:, :nx]
        plan_states, should_unfold = unfold_first_wall_bounce(plan_states, tf, time_into_pp)
        horizon = jnp.maximum(tf - time_into_pp, 1e-6)
        state_sample_times = time_into_pp + jnp.linspace(
            horizon / H_mpc, horizon, H_mpc
        )

        x_ref_future = interpolate_reference(plan_states, tf, state_sample_times)
        # First state of reference trajectory doesnt matter for optimization anyway (Equality constraint forces x[0] = x0)
        x_ref = jnp.vstack([x0[start_idx : start_idx + nx], x_ref_future])

        my_state = x0[start_idx : start_idx + nx]
        P, c, A, b, G, h = build_qp(my_state, x_ref, horizon)

        sol = solver.run(
            params_obj=(P, c),
            params_eq=(A, b),
            params_ineq=(G, h),
        ).params

        x_seq = sol[0][: (H_mpc + 1) * nx].reshape(H_mpc + 1, nx)
        x_final = x_seq[-1]
        delta_pos = jnp.linalg.norm(x_final[:2] - x_ref[-1, :2])
        delta_vel = jnp.linalg.norm(x_final[3:6] - x_ref[-1, 3:6])

        jax.debug.print("MPC has bounce: {}", should_unfold)
        u_seq = sol[0][(H_mpc + 1) * nx :].reshape(H_mpc, nu)

        u0 = u_seq[0]
        return u0
    
    def final_only_controller(x0, x_target, tf, time_into_pp):
        horizon = jnp.maximum(tf - time_into_pp, 1e-6)
        
        x_ref = jnp.vstack([x0[start_idx : start_idx + nx], jnp.tile(x_target, (H_mpc, 1))])
        my_state = x0[start_idx : start_idx + nx]
        P, c, A, b, G, h = build_qp(my_state, x_ref, horizon)

        sol = solver.run(
            params_obj=(P, c),
            params_eq=(A, b),
            params_ineq=(G, h),
        ).params
        x_seq = sol[0][: (H_mpc + 1) * nx].reshape(H_mpc + 1, nx)
        x_final = x_seq[-1]
        # jax.debug.print("MPC final-only controller terminal state: {}", x_final)
        u_seq = sol[0][(H_mpc + 1) * nx :].reshape(H_mpc, nu)
        u0 = u_seq[0]
        return u0

    if not final_only:
        return jax.jit(controller)
    else:
        return jax.jit(final_only_controller)
