"""
DIAL controller (Till's project). Robot 1 attacks from the start: every 1 / dial.path_planning_hz
the planner (planner.py) re-plans its input sequence from the current state, and in between the
plan's inputs are interpolated. Once robot 1 touches the puck it defends at home with a PD
controller for the rest of the episode.

The plan is H input nodes spread over its duration tf; tf is a decision variable of the planner.
The planner imagines robot 2 as a force field (controllers.pd.build_force_field_enemy_controller).
"""

import numpy as np
import jax
import jax.numpy as jnp

from controllers import DEF, ATT
from controllers.pd import build_force_field_enemy_controller, defence_controller, home_state
from .planner import build_dial_path_planner
from .rollout import rollout, rollout_pd


def build_interpolation_controller(H):
    """Input at time time_pp into a plan of duration tf, linearly interpolated between its nodes."""
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


class DialFactory:
    """Builds the jitted planner and controllers once per process (on the first episode): each
    jax.jit compiles on its first call."""

    def __init__(self, config):
        self.config = config
        self.parts = None

    def new(self, physics, init_state, verbose=False):
        if self.parts is None:
            config = self.config
            r2_imagined = build_force_field_enemy_controller(
                home_state=jnp.array(home_state(config, 2)),
                table_width=config["table"]["width"],
                robot_id=2,
            )
            def_controller = defence_controller(config, 1)
            self.parts = dict(
                planner=build_dial_path_planner(config, def_controller, r2_imagined),
                interpolate=build_interpolation_controller(config["dial"]["H_interpolation"]),
                def_controller=def_controller,
                r2_imagined=r2_imagined,
            )
        return DialController(self.config, physics, init_state, **self.parts)

    def summary(self, logs):
        return ""

    def close(self):
        pass


class DialController:
    def __init__(self, config, physics, init_state, planner, interpolate, def_controller, r2_imagined):
        d = config["dial"]
        self.physics = physics
        self.planner, self.interpolate, self.def_controller = planner, interpolate, def_controller
        self.H = H = d["H"]
        self.dt = 1.0 / config["simulation"]["hz"]
        self.dt_pp = 1.0 / d["path_planning_hz"]
        self.replan_every = int(round(self.dt_pp / self.dt))
        self.u_max = np.array(config["simulation"]["u_limits"])
        n_goal_check = int(round(d["goal_check_horizon"] / self.dt))

        self.key = jax.random.PRNGKey(1)
        self.mode = ATT

        # Initial plan: a PD rollout toward where the puck will be after the time robot 1 needs to
        # reach it at full acceleration
        x0 = jnp.asarray(init_state, dtype=jnp.float32)
        a_max = physics.robot.u_limits[0] / physics.robot.mass
        r1p_dist = jnp.linalg.norm(x0[:2] - x0[12:14])
        delta_v = jnp.linalg.norm(x0[3:5] - x0[15:17])
        # s = (1/2) * a * t^2 + v * t
        tf = -delta_v / a_max + jnp.sqrt((delta_v / a_max) ** 2 + 2 * r1p_dist / a_max)
        no_r1 = x0.at[:6].set(0.0)
        puck_traj, _ = rollout(no_r1, jnp.zeros((H, 6)), physics, tf / H, r2_imagined)
        traj, inputs = rollout_pd(x0, puck_traj[-1, 12:15], H, physics, tf / H, r2_imagined)
        inputs = jnp.clip(inputs, -self.u_max, self.u_max)

        self.plan = jnp.hstack((traj[:, :6], inputs))  # (H, 9): states, inputs
        self.tf = tf
        self.future = jnp.zeros((H + n_goal_check, 18))  # predicted states (plan, then shot), for display
        self.diffusion_trajs = jnp.zeros((d["N"], d["n_debug_trajectories"], H, 2))  # robot xy of some samples
        self.diffusion_costs = jnp.zeros((d["N"], d["n_debug_trajectories"])).at[:, 0].set(1.0)

    def step(self, i, state):
        if self.mode == ATT:
            if i % self.replan_every == 0:
                self._replan(state)
            time_pp = (i % self.replan_every) * self.dt  # how far down the plan we are
            u = self.interpolate(state, self.plan, self.tf, time_pp)
        else:
            u = self.def_controller(state)
        return np.clip(np.asarray(u, dtype=float), -self.u_max, self.u_max)

    def _replan(self, state):
        """New plan, warm-started from the current one shifted by the time since it was made."""
        roll_by = jnp.round(self.dt_pp / (self.tf / self.H)).astype(int)
        rolled = self.plan[jnp.minimum(jnp.arange(self.H) + roll_by, self.H - 1)]
        (self.key, self.plan, self.tf, self.future,
         self.diffusion_trajs, self.diffusion_costs) = self.planner(self.key, state, rolled, self.tf, self.physics)

    def notify_contact(self, t):
        self.mode = DEF  # for the rest of the episode

    def record(self, state, t):
        return {"path_plans": np.asarray(self.plan), "tfs": float(self.tf), "future_trajs": np.asarray(self.future),
                "diffusion_trajs": np.asarray(self.diffusion_trajs), "diffusion_costs": np.asarray(self.diffusion_costs)}

    def log(self):
        return {}
