"""
Low-level controllers of DIAL, after Till's controller.py:
- the path follower, an MPC that tracks the planned states up to the impact (Till's report, 3.3);
- the LQR that holds robot 1 at home while no shot is planned (DEF).
"""

import numpy as np
import scipy.linalg
import jax
import jax.numpy as jnp
import jaxopt
import interpax

from .model import discrete_system


def build_path_follower(config, physics, final_only=False):
    """
    MPC over dial.H_mpc steps spread over the time left to the impact: tracks the plan's states
    (interpolated by a spline) and must reach the planned impact position at the end (a hard
    constraint; the terminal state also weighs dial.Qt_factor Q). Its model is the planning model
    without walls and collisions: a plan that bounces robot 1 off a wall is mirrored about that
    wall instead (report, Fig. 6).
    Returns controller(state, plan, tf, time_into_plan) -> robot 1 input, or with final_only
    controller(state, target_state, tf, time_into_plan), which only tracks a final state.
    """
    d = config["dial"]
    H_mpc = d["H_mpc"]
    Q = jnp.diag(jnp.array(d["Q_diag"]))
    R = jnp.diag(jnp.array(d["R_diag"]))
    Q_terminal = d["Qt_factor"] * Q
    nx, nu = 6, 3
    u_max = jnp.array(config["simulation"]["u_limits"])
    solver = jaxopt.OSQP(tol=1e-3, maxiter=2000)

    x_min, x_max = physics.robot.radius, physics.table.width - physics.robot.radius
    y_min, y_max = physics.robot.radius, physics.table.height - physics.robot.radius
    wall_hit_margin = 2.0 * physics.robot.radius

    def mirror_vertical(plan_states, wall_x):
        return plan_states.at[:, 0].set(2.0 * wall_x - plan_states[:, 0]).at[:, 3].set(-plan_states[:, 3])

    def mirror_horizontal(plan_states, wall_y):
        return plan_states.at[:, 1].set(2.0 * wall_y - plan_states[:, 1]).at[:, 4].set(-plan_states[:, 4])

    def unfold_first_wall_bounce(plan_states, tf, time_into_plan):
        """The plan with its states after the first wall bounce still ahead mirrored about that wall."""
        pos, vel = plan_states[:, :2], plan_states[:, 3:5]
        n = plan_states.shape[0]
        idx = jnp.arange(n - 1)
        never = n  # larger than any bounce step

        def near(axis, bound, below):
            return ((pos[:-1, axis] <= bound + wall_hit_margin) | (pos[1:, axis] <= bound + wall_hit_margin) if below
                    else (pos[:-1, axis] >= bound - wall_hit_margin) | (pos[1:, axis] >= bound - wall_hit_margin))

        hits = [
            (vel[:-1, 0] < 0.0) & (vel[1:, 0] > 0.0) & near(0, x_min, True),  # left
            (vel[:-1, 0] > 0.0) & (vel[1:, 0] < 0.0) & near(0, x_max, False),  # right
            (vel[:-1, 1] < 0.0) & (vel[1:, 1] > 0.0) & near(1, y_min, True),  # bottom
            (vel[:-1, 1] > 0.0) & (vel[1:, 1] < 0.0) & near(1, y_max, False),  # top
        ]
        steps = jnp.array([jnp.min(jnp.where(hit, idx, never)) for hit in hits])
        wall = jnp.argmin(steps)
        bounce_step = steps[wall]
        ahead = (bounce_step < never) & (bounce_step * tf / jnp.maximum(n, 1) > time_into_plan)

        mirrored = jax.lax.switch(wall, (
            lambda s: mirror_vertical(s, x_min),
            lambda s: mirror_vertical(s, x_max),
            lambda s: mirror_horizontal(s, y_min),
            lambda s: mirror_horizontal(s, y_max),
        ), plan_states)
        unfolded = jnp.where((jnp.arange(n) > bounce_step)[:, None], mirrored, plan_states)
        return jnp.where(ahead, unfolded, plan_states)

    def interpolate_reference(plan_states, tf, sample_times):
        """Plan states at the given times; row k of the plan is the state at (k + 1) tf / H."""
        plan_times = jnp.arange(0.0, 1.0 + 1e-8, 1.0 / plan_states.shape[0])[1:]
        spline = interpax.CubicSpline(plan_times, plan_states, axis=0, check=False)
        return spline(jnp.clip(sample_times / jnp.maximum(tf, 1e-8), 0.0, 1.0))

    def solve_qp(x0, x_ref, horizon):
        """First input of the MPC from x0 over horizon toward the references x_ref[1:]."""
        Ad, Bd = discrete_system(physics, horizon / H_mpc)

        # Equality constraints: x_0 = x0, x_{k+1} = A_d x_k + B_d u_k, and the position of x_H is the
        # reference's; variables (x_0..x_H, u_0..u_{H-1})
        A_x = jnp.eye(nx * (H_mpc + 1)) + jnp.vstack([
            jnp.zeros((nx, nx * (H_mpc + 1))),
            jnp.hstack([jnp.kron(jnp.eye(H_mpc), -Ad), jnp.zeros((H_mpc * nx, nx))]),
        ])
        A_u = jnp.zeros((nx * (H_mpc + 1), H_mpc * nu)).at[nx:, :].set(jnp.kron(jnp.eye(H_mpc), -Bd))
        A_terminal = jnp.zeros((2, nx * (H_mpc + 1) + H_mpc * nu)).at[:, H_mpc * nx:H_mpc * nx + 2].set(jnp.eye(2))
        A_eq = jnp.vstack([jnp.hstack([A_x, A_u]), A_terminal])
        b_eq = jnp.concatenate([x0, jnp.zeros(H_mpc * nx), x_ref[-1, :2]])

        # |u| <= u_limits
        G = jnp.vstack([jnp.eye(nu), -jnp.eye(nu)])
        G_qp = jnp.hstack([jnp.zeros((2 * nu * H_mpc, nx * (H_mpc + 1))), jnp.kron(jnp.eye(H_mpc), G)])
        h_qp = jnp.tile(jnp.concatenate([u_max, u_max]), H_mpc)

        P = jax.scipy.linalg.block_diag(*([Q] * H_mpc + [Q_terminal] + [R] * H_mpc))
        c = jnp.concatenate([-jnp.reshape(x_ref[:-1] @ Q.T, (-1,)), -Q_terminal @ x_ref[-1], jnp.zeros(H_mpc * nu)])

        sol = solver.run(params_obj=(P, c), params_eq=(A_eq, b_eq), params_ineq=(G_qp, h_qp)).params
        return sol[0][(H_mpc + 1) * nx:].reshape(H_mpc, nu)[0]

    def controller(x0, plan, tf, time_into_plan):
        plan_states = unfold_first_wall_bounce(plan[:, :nx], tf, time_into_plan)
        horizon = jnp.maximum(tf - time_into_plan, 1e-6)
        sample_times = time_into_plan + jnp.linspace(horizon / H_mpc, horizon, H_mpc)
        # x_ref[0] does not count: the equality constraints set x_0 = x0
        x_ref = jnp.vstack([x0[:nx], interpolate_reference(plan_states, tf, sample_times)])
        return solve_qp(x0[:nx], x_ref, horizon)

    def final_only_controller(x0, target_state, tf, time_into_plan):
        horizon = jnp.maximum(tf - time_into_plan, 1e-6)
        x_ref = jnp.vstack([x0[:nx], jnp.tile(target_state, (H_mpc, 1))])
        return solve_qp(x0[:nx], x_ref, horizon)

    return jax.jit(final_only_controller if final_only else controller)


def build_lqr_controller(target_state, config, physics):
    """Discrete LQR at the control rate holding robot 1 at target_state, on the double integrator
    (no friction) discretised with a zero-order hold."""
    dt = 1.0 / config["simulation"]["hz"]
    M = np.diag([1 / physics.robot.mass, 1 / physics.robot.mass, 1 / physics.robot.inertia])
    Ad = np.block([[np.eye(3), dt * np.eye(3)], [np.zeros((3, 3)), np.eye(3)]])
    Bd = np.vstack([0.5 * dt**2 * M, dt * M])
    Q = np.diag([10.0, 10.0, 10.0, 0.5, 0.5, 0.5])
    R = np.diag([0.01, 0.01, 0.05])
    P = scipy.linalg.solve_discrete_are(Ad, Bd, Q, R)
    K = jnp.array(np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad))
    target = jnp.asarray(target_state)

    def controller(x0):
        return K @ (target - x0[:6])

    return jax.jit(controller)
