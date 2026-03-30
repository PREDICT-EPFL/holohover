import jax
import jax.numpy as jnp
import jaxopt
import numpy as np
import scipy.signal
from scipy.linalg import block_diag


def build_interpolation_controller(config):
    H = config["simulation"]["H_mpc"]

    def controller(x0, plan, tf, time_pp):
        t_normalized = time_pp / tf * H # in range [0, H]
        idx = jnp.floor(t_normalized).astype(int)
        idx = jnp.clip(idx, 0, H-1)

        u_left = plan[idx, 6:]
        u_right = jnp.where(idx<H-1, plan[idx+1, 6:], jnp.zeros(3))

        # interpolate
        alpha = jnp.clip(t_normalized - idx, 0, 1)
        u0 = (1-alpha) * u_left + alpha * u_right

        u0 = jnp.where(time_pp > tf, plan[H-1, 6:], u0)

        return u0
    
    return jax.jit(controller)


def build_pd_controller(config, physics):
    H_mpc = config["simulation"]["H_mpc"]
    target_pos = physics.table.defensive_position
    target_vel = np.array([0.0, 0.0, 0.0])

    def controller(x0):
        pos = x0[:3]
        vel = x0[3:6]
        Kp = 5.0
        Kd = 1.0
        pos_error = target_pos - pos
        vel_error = target_vel - vel

        u0 = Kp * pos_error + Kd * vel_error
        return u0

    return jax.jit(controller)


def build_mpc_controller(config, physics):
    H_mpc = config["simulation"]["H_mpc"]
    dt = 1 / config["simulation"]["hz"]

    Q = jnp.diag(jnp.array(config["dial"]["Q_diag"]))
    R = jnp.diag(jnp.array(config["dial"]["R_diag"]))

    nx = 6
    nu = 3

    A = physics.robot.A
    B = physics.robot.B
    dt = 1 / config["simulation"]["hz"]

    C = jnp.zeros((1, nx)) # useless filler for cont2discrete
    D = jnp.zeros((1, nu))

    Ad, Bd, _, _, _ = scipy.signal.cont2discrete(system=(A, B, C, D), dt=dt)
    Ad = jnp.array(Ad)
    Bd = jnp.array(Bd)

    def build_qp(x0, xt):
        nx, nu = B.shape

        A_qp_left = jnp.kron(jnp.eye(H_mpc+1), jnp.eye(nx))
        A_qp_left = A_qp_left + jnp.vstack([
            jnp.zeros((nx, nx*(H_mpc+1))),
            jnp.hstack([jnp.kron(jnp.eye(H_mpc), -Ad), jnp.zeros((H_mpc*nx, nx))])
        ])
        A_qp_right = jnp.zeros((A_qp_left.shape[0], H_mpc*nu))
        A_qp_right = A_qp_right.at[nx:, :].set(A_qp_right[nx:, :] + jnp.kron(jnp.eye(H_mpc), -Bd))
        A_qp = jnp.hstack([A_qp_left, A_qp_right])

        b_qp = jnp.concatenate([x0, jnp.zeros((H_mpc)*nx)])

        # Inequality constraints G*x <= h
        G = np.array([[0, 0, 0, 0, 0, 0,   1, 0, 0],
                       [0, 0, 0, 0, 0, 0,   -1, 0, 0],
                       [0, 0, 0, 0, 0, 0,   0, 1, 0],
                       [0, 0, 0, 0, 0, 0,   0, -1, 0]])
        h = jnp.array([20, 20, 20, 20])
        G_blocks = [np.kron(np.eye(H_mpc), G)]
        G_qp = jnp.array(block_diag(*G_blocks))

        h_qp = jnp.concatenate([
            jnp.kron(jnp.ones((H_mpc, 1)), h).reshape(-1,),
        ])

        # Cost function 1/2*x'*P*x
        P_blocks = [np.kron(np.eye(H_mpc+1), Q), np.kron(np.eye(H_mpc), R)]
        P_qp = jnp.array(block_diag(*P_blocks))

        c = jnp.zeros(nx + H_mpc * (nx + nu))
        # Fill c with -Q @ xt for tracking error
        xt_flat = xt.reshape(-1, order="F")
        # jax.debug.print("xt {}", xt_flat)
        c_qp = c.at[:(H_mpc+1) * nx].set(-(jnp.kron(jnp.eye(H_mpc+1), Q) @ xt_flat))

        return P_qp, c_qp, A_qp, b_qp, G_qp, h_qp

    def interpolate_reference(x0, plan, tf, time_pp):
        # Vectorized interpolation of reference trajectory
        t_normalized = (time_pp + jnp.arange(H_mpc + 1) * dt) / tf * H_mpc
        idx = jnp.floor(t_normalized).astype(int)
        idx = jnp.clip(idx, 0, H_mpc - 1)

        # Extract p0, p1, v0, v1 for all steps at once
        p0_rep = jnp.repeat(x0[:3].reshape((1,3)), repeats=H_mpc+1, axis=0)
        v0_rep = jnp.repeat(x0[3:6].reshape((1,3)), repeats=H_mpc+1, axis=0)
        p0 = jnp.where(idx.reshape((H_mpc+1,1)) != 0, plan[idx - 1, :3], p0_rep)
        p1 = jnp.where(idx.reshape((H_mpc+1,1)) != H_mpc - 1, plan[idx, :3], plan[H_mpc - 1, :3])
        v0 = jnp.where(idx.reshape((H_mpc+1,1)) != 0, plan[idx - 1, 3:6], v0_rep)
        v1 = jnp.where(idx.reshape((H_mpc+1,1)) != H_mpc - 1, plan[idx, 3:6], plan[H_mpc - 1, 3:6])

        alpha = t_normalized - idx
        target_pos = (1 - alpha[:, None]) * p0 + alpha[:, None] * p1
        target_vel = (1 - alpha[:, None]) * v0 + alpha[:, None] * v1

        xt = jnp.zeros((nx, H_mpc + 1))
        xt = xt.at[:3, :].set(target_pos.T)
        xt = xt.at[3:6, :].set(target_vel.T)
        return xt

    def controller(x0, plan, tf, time_pp):
        xt = interpolate_reference(x0[:6], plan, tf, time_pp)
        # jax.debug.print("xt {}", xt)
        P, c, A, b, G, h = build_qp(x0[:6], xt)

        solver = jaxopt.OSQP()
        sol = solver.run(params_obj=(P, c),
                         params_eq=(A, b),
                        #  params_ineq=(G, h),
                         ).params
        # jax.debug.print("b {}", sol)
        u_seq = sol[0][(H_mpc+1) * nx :].reshape(H_mpc, nu)
        u0 = u_seq[0]
        return u_seq, u0

    return jax.jit(controller)
