"""
One closed-loop episode, the same for every controller. At each control step (simulation.hz):
robot 1's controller gives its input, robot 2 holds its home with a PD controller (opponent
"static") or stays parked (opponent "none"), then simulation.physics_hz / hz physics substeps run
under the held inputs.

The recorded data (episode_data) is what viz/ reads: one row per step, row 0 the initial state.
"""

from functools import partial

import numpy as np
import jax
import jax.numpy as jnp

from controllers import ATT
from controllers.pd import defence_controller
from .dynamics import physics_step


def n_substeps(config):
    """Physics steps per control step. The simulator only detects a robot-puck contact at the end
    of a step, and takes the contact normal from the overlapping centres: at 100 Hz a fast strike
    overlaps by ~2 cm and the shot leaves several degrees off. Physics at 500 Hz or more keeps that
    error below half a degree."""
    sim = config["simulation"]
    n = sim.get("physics_hz", sim["hz"]) / sim["hz"]
    assert n >= 1 and n == int(n), "simulation.physics_hz must be a multiple of simulation.hz"
    return int(n)


@partial(jax.jit, static_argnums=4)
def physics_substeps(state, control, physics, h, n):
    """n physics steps of length h under one held control; per-substep contact and goal flags."""
    def body(s, _):
        s, r1_col, _, has_scored, _, _ = physics_step(s, control, physics, h)
        return s, (r1_col, has_scored)

    state, (r1_cols, scored) = jax.lax.scan(body, state, None, length=n)
    return state, r1_cols, scored


class Episode:
    """One closed-loop run, advanced one control step at a time."""

    def __init__(self, config, physics, controller, init_state):
        self.ctrl = controller
        self.physics = physics
        self.hz = config["simulation"]["hz"]
        self.dt = 1.0 / self.hz
        self.n_sub = n_substeps(config)
        self.h = self.dt / self.n_sub
        self.F_max = np.array(config["simulation"]["u_limits"])
        self.r2_ctrl = defence_controller(config, 2) if config["simulation"]["opponent"] == "static" else None

        self.state = np.asarray(init_state, dtype=float)
        self.score = np.zeros(2, dtype=int)
        self.i = 0  # control steps taken; the simulation time is i * dt
        self.contacts = []  # times robot 1 touched the puck
        self.outcome, self.t_goal, self.i_goal, self.contacts_at_goal = None, None, None, None  # first goal
        self.attacked = False

    def step(self, record=True):
        """One control step, then the physics substeps. With record, returns the step's row:
        the applied input, the mode, the controller's record, then the new state and score."""
        ctrl, state, i = self.ctrl, self.state, self.i
        t = i * self.dt
        F1 = ctrl.step(i, state)
        F2 = (np.clip(np.asarray(self.r2_ctrl(jnp.asarray(state)), dtype=float), -self.F_max, self.F_max)
              if self.r2_ctrl else np.zeros(3))
        self.attacked |= ctrl.mode == ATT
        row = {"input_sequence": F1, "modes": ctrl.mode, **ctrl.record(state, t)} if record else None

        u = jnp.asarray(np.concatenate([F1, F2]), dtype=jnp.float32)
        new_state, r1_cols, scored = physics_substeps(jnp.asarray(state, dtype=jnp.float32), u, self.physics,
                                                      self.h, self.n_sub)
        self.state = np.asarray(new_state, dtype=float)
        r1_cols, scored = np.asarray(r1_cols), np.asarray(scored)
        if r1_cols.any():
            t_contact = t + (np.argmax(r1_cols) + 1) * self.h
            self.contacts.append(t_contact)
            ctrl.notify_contact(t_contact)
        j_goal = np.flatnonzero(scored)
        has_scored = int(scored[j_goal[0]]) if len(j_goal) else 0
        if has_scored:
            self.score[has_scored - 1] += 1
            if self.outcome is None:
                self.outcome = "goal" if has_scored == 1 else "own_goal"
                self.t_goal, self.i_goal = t + (j_goal[0] + 1) * self.h, i
                self.contacts_at_goal = len(self.contacts)
        self.i += 1

        if record:
            row["states"] = self.state.copy()
            row["scores"] = self.score.copy()
        return row

    def final_outcome(self):
        """The first goal's outcome, or why there was none."""
        if self.outcome is not None:
            return self.outcome
        return "miss" if self.contacts else ("no_contact" if self.attacked else "no_attack")


def run_episode(config, physics, controller, init_state, duration, stop_on_goal=False, record=True, tail=0.0):
    """Run an episode for `duration` seconds; with stop_on_goal, it ends `tail` seconds after the first goal."""
    ep = Episode(config, physics, controller, init_state)
    tail_steps = int(round(tail * ep.hz))
    rec = {}
    for _ in range(int(round(duration * ep.hz))):
        row = ep.step(record)
        if record:
            for key, value in row.items():
                rec.setdefault(key, []).append(value)
        if stop_on_goal and ep.i_goal is not None and ep.i - 1 >= ep.i_goal + tail_steps:
            break

    result = {
        "outcome": ep.final_outcome(),
        "t_goal": ep.t_goal,
        "contacts": ep.contacts,
        "contacts_at_goal": ep.contacts_at_goal,
        "log": controller.log(),
    }
    if record:
        result["data"] = episode_data(config, init_state, rec, getattr(controller, "candidate_labels", ()))
    return result


def episode_log(result):
    """JSON-able log of an episode: its outcome and contacts, plus the controller's log."""
    return {"outcome": result["outcome"], "t_goal": result["t_goal"], "contacts": result["contacts"],
            **result["log"]}


def episode_data(config, init_state, rec, candidate_labels=()):
    """
    The recorded rows (Episode.step) as arrays, one row per step plus row 0, the initial state:
    there the inputs and the controller's data repeat those of the first step.
        states (n, 18), scores (n, 2), input_sequence (n, 3), modes (n,), and from the controller:
        path_plans (n, H, 9) robot 1's plan as rows (x, y, theta, vx, vy, omega, Fx, Fy, tau),
        tfs (n,), future_trajs (n, m, 18) predicted states (robot 2 and puck are drawn), plus
        shot_mpc: plan_input_sequence, def_input_sequence and candidates (a dict, see viz/animation.py)
        dial: diffusion_trajs (n, N, n_debug, H, 2) and diffusion_costs (n, N, n_debug)
    """
    data = {key: np.concatenate([np.asarray(rows[:1]), np.asarray(rows)]) for key, rows in rec.items()}
    data["states"][0] = init_state
    data["scores"][0] = 0
    candidates = {key[len("cand_"):]: data.pop(key) for key in list(data) if key.startswith("cand_")}
    if candidates:
        data["candidates"] = {"labels": list(candidate_labels), **candidates}
    data["config"] = config
    return data
