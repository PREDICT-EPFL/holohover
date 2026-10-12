"""
DIAL controller: Till's final DIAL (Till-mpc-semester-project, branch pvp, commit 35c96ab;
SP_hockey/literature/report-till.pdf) for robot 1.

Every 1 / dial.path_planning_hz the planner (planner.py) plans a shot from the current state:
- in DEF, from a fresh initial guess (initial_plan), while an LQR holds robot 1 at home;
- in ATT, warm-started from the previous plan shifted by the planning period (warm_start).
Robot 1 switches to ATT when a plan is a valid shot, and then follows it with the MPC path
follower (follower.py); it switches back to DEF when it touches the puck. The planner imagines
robot 2 standing still (no force).

Differences from Till's code: the plant is simulation/dynamics.py while DIAL plans with his
physics (model.py), and there is no planning delay (his path_planning_delay_ms).
"""

import numpy as np
import jax
import jax.numpy as jnp
import interpax

from controllers import DEF, ATT
from controllers.pd import home_state
from .follower import build_lqr_controller, build_path_follower
from .planner import build_dial_path_planner
from .rollout import rollout, rollout_mpc


class DialFactory:
    """Builds the jitted planner, follower and helpers once per process (on the first episode):
    each jax.jit compiles on its first call."""

    def __init__(self, config):
        self.config = config
        self.parts = None

    def new(self, physics, init_state, verbose=False):
        if self.parts is None:
            self.parts = build_parts(self.config, physics)
        return DialController(self.config, physics, **self.parts)

    def summary(self, logs):
        return ""

    def close(self):
        pass


def build_parts(config, physics):
    d = config["dial"]
    H, K = d["H"], d["n_knots"]
    dt_pp = 1.0 / d["path_planning_hz"]
    a_max = config["simulation"]["u_limits"][0] / config["robot"]["mass"]
    knot_times = jnp.linspace(0.0, 1.0, K + 1, endpoint=False)
    control_times = jnp.linspace(0.0, 1.0, H, endpoint=False)
    spline_bc = ((1, jnp.zeros(3)), "not-a-knot")  # zero slope at the start

    def enemy(x):
        return jnp.zeros(3)  # robot 2 imagined still

    def_controller = build_lqr_controller(home_state(config, 1), config, physics)
    reach = build_path_follower(config, physics, final_only=True)

    @jax.jit
    def initial_plan(state):
        """tf: the time robot 1 needs, at full acceleration along x, to meet the puck (at most 1 s);
        inputs: the follower driving robot 1 to where the puck will be at tf, fitted with the spline."""
        rel_dist = jnp.abs(state[0] - state[12])
        delta_v = state[3] - state[15]
        tf = jnp.minimum(delta_v / a_max + jnp.sqrt((delta_v / a_max) ** 2 + 2 * rel_dist / a_max), 1.0)
        puck_traj, _, _ = rollout(state.at[:6].set(0.0), jnp.zeros((H, 3)), physics, tf / H, enemy)
        _, inputs = rollout_mpc(state, puck_traj[-1, 12:15], H, physics, tf / H, enemy, reach)
        knots = interpax.CubicSpline(control_times, inputs, axis=0, bc_type=spline_bc, check=False)(knot_times)
        return tf, interpax.CubicSpline(knot_times, knots, axis=0, bc_type=spline_bc, check=False)

    @jax.jit
    def warm_start(spline, tf, cost):
        """The plan dt_pp later: the spline shifted by dt_pp and re-sampled over [0, tf - dt_pp],
        with its slope kept at the splice."""
        tf_new = jnp.maximum(tf - dt_pp, 1e-8)

        def shift(_):
            times = jnp.clip((dt_pp + knot_times * tf_new) / jnp.maximum(tf, 1e-8), 0.0, 1.0)
            rate = spline.derivative()(times[0]) * (tf_new / jnp.maximum(tf, 1e-8))
            shifted = interpax.CubicSpline(knot_times, spline(times), axis=0, bc_type=((1, rate), "not-a-knot"),
                                           check=False)
            return shifted, tf_new, cost - 1e1 * tf**2 + 1e1 * tf_new**2

        return jax.lax.cond(tf_new > 1e-8, shift, lambda _: (spline, tf, jnp.inf), operand=None)

    return dict(
        planner=build_dial_path_planner(config, def_controller, enemy),
        follower=build_path_follower(config, physics),
        def_controller=def_controller,
        initial_plan=initial_plan,
        warm_start=warm_start,
    )


class DialController:
    def __init__(self, config, physics, planner, follower, def_controller, initial_plan, warm_start):
        d = config["dial"]
        self.physics = physics
        self.planner, self.follower, self.def_controller = planner, follower, def_controller
        self.initial_plan, self.warm_start = initial_plan, warm_start
        self.dt = 1.0 / config["simulation"]["hz"]
        self.replan_every = int(round(config["simulation"]["hz"] / d["path_planning_hz"]))
        self.u_max = np.array(config["simulation"]["u_limits"])
        H, n_goal_check = d["H"], int(round(d["goal_check_horizon"] * d["goal_check_hz"]))

        self.key = jax.random.PRNGKey(0)
        self.mode = DEF
        self.attacking = False  # mode during the current step: a contact in DEF does not count
        self.valid_shot = False  # the last plan is a valid shot
        self.spline, self.tf, self.cost = None, 0.0, np.inf
        self.plan = np.zeros((H, 9))  # (H, 9): planned states and inputs
        self.future = np.zeros((H + n_goal_check, 18))  # planned states, then the shot
        self.diffusion_trajs = np.zeros((d["N"], d["n_debug_trajectories"], H, 2))
        self.diffusion_costs = np.zeros((d["N"], d["n_debug_trajectories"]))
        self.plan_history = []  # (t, "dial", tf) of every plan that is a valid shot

    def step(self, i, state):
        """Robot 1's input at control step i (world frame, clipped)."""
        self.attacking = self.mode == ATT
        replan = i % self.replan_every == 0
        if replan:
            x = jnp.asarray(state, dtype=jnp.float32)
            if self.mode == DEF:
                tf, spline = self.initial_plan(x)
                cost = jnp.inf
            else:
                spline, tf, cost = self.warm_start(self.spline, self.tf, self.cost)
            (self.key, self.spline, self.plan, self.tf, self.cost, self.future, self.diffusion_trajs,
             self.diffusion_costs, valid_shot) = self.planner(self.key, x, spline, tf, cost, self.physics)
            self.valid_shot = bool(valid_shot)
            if self.valid_shot:
                self.plan_history.append((i * self.dt, "dial", float(self.tf)))

        if self.attacking:
            u = self.follower(state, self.plan, self.tf, (i % self.replan_every) * self.dt)
        else:
            u = self.def_controller(state)
        if replan and self.mode == DEF and self.valid_shot:
            self.mode = ATT  # from the next step
        return np.clip(np.asarray(u, dtype=float), -self.u_max, self.u_max)

    def notify_contact(self, t):
        if self.attacking:
            self.mode = DEF

    def record(self, state, t):
        return {"path_plans": np.asarray(self.plan), "tfs": float(self.tf), "future_trajs": np.asarray(self.future),
                "diffusion_trajs": np.asarray(self.diffusion_trajs),
                "diffusion_costs": np.asarray(self.diffusion_costs)}

    def log(self):
        return {"plan_history": self.plan_history}
