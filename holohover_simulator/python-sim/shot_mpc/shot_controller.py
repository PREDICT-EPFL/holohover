"""
Closed-loop integration (Section 8, direct mode): P(B*) runs as the MPC itself.

- Planning ticks at inner_hz re-solve P(B*); every 1/outer_hz all sequences are
  re-solved and B* is re-selected (unless committed near impact).
- Between ticks the tracking QP follows the plan (shot_mpc.tracker) when
  shot_mpc.tracker.enabled; otherwise the plan's zero-order-hold input is applied.
  The plan has no heading axis: only the tracker controls the heading.
- DEF/ATT state machine as in Till's project: defend at home while no valid plan
  exists, attack once one does, return to defence after contact.
"""

import numpy as np
import jax.numpy as jnp

from controller import build_pd_controller

from .params import (
    ShotParams, XY, state_world_to_planner, state_planner_to_world,
    accel_planner_to_force_world, force_world_to_accel_planner, pos_planner_to_world,
)
from .model import predict, bounce_path, shot_polyline
from .shot_nlp import ShotNLP
from .shot_selector import ShotSelector
from .tracker import Tracker

DEF, ATT = 0, 1


class ShotController:
    def __init__(self, config, physics, P: ShotParams = None, nlp: ShotNLP = None, verbose=True):
        self.P = P or ShotParams.from_config(config)
        self.nlp = nlp or ShotNLP(self.P, verbose=verbose)
        self.selector = ShotSelector(self.P, self.nlp)
        trk = config["shot_mpc"].get("tracker", {})
        self.tracker = Tracker(self.P, trk) if trk.get("enabled") else None
        self.verbose = verbose

        hz = config["simulation"]["hz"]
        self.dt = 1.0 / hz
        self.inner_every = max(1, int(round(hz / self.P.inner_hz)))
        self.outer_every = max(self.inner_every, int(round(hz / self.P.outer_hz)))
        self.F_max = np.array(config["simulation"]["u_limits"])

        home = np.array([config["robot"]["robot1"]["defensive_position_rel"] * physics.table.width,
                         0.5 * physics.table.height, 0.0, 0.0, 0.0, 0.0])
        self.def_controller = build_pd_controller(jnp.array(home), robot_id=1)

        self.mode = DEF
        self.plan = None
        self.puck = None  # puck flight predicted at the last planning tick
        self.u_prev = np.zeros(3)  # last applied input, planner accelerations (a_x, a_y, a_theta)
        self.attack_after = 0.0
        self.contact_times = []
        self.plan_history = []  # (t, label, tf) of every accepted plan

    def reset_plan(self):
        self.plan = None
        self.selector.reset()

    def notify_contact(self, t):
        """Robot 1 touched the puck: back to defence for a cooldown (Till's state machine)."""
        self.contact_times.append(t)
        self.mode = DEF
        self.reset_plan()
        self.attack_after = t + self.P.contact_cooldown

    def step(self, i, state):
        """Return the world-frame input (Fx, Fy, tau) of robot 1 at simulation step i."""
        P = self.P
        t = i * self.dt
        state = np.asarray(state, dtype=float)
        xR = state_world_to_planner(state[0:6], P)  # (x, y, theta, vx, vy, omega); the planner uses xR[XY]
        puck = state_world_to_planner(state[12:18], P)

        # Drop a plan whose impact passed without contact
        if self.plan is not None and self.plan.remaining(t) < -P.plan_expiry:
            if self.verbose:
                print(f"t={t:.2f}s plan {self.plan.label} expired without contact")
            self.mode = DEF
            self.reset_plan()

        outer_tick = i % self.outer_every == 0
        # Without a plan (DEF) the sequences are searched at the outer rate only
        if t >= self.attack_after and i % self.inner_every == 0 and (self.plan is not None or outer_tick):
            self.puck = predict(puck[0:2], puck[3:5], P)
            remaining = self.plan.remaining(t) if self.plan is not None else np.inf
            if remaining > P.t_stop_replan:
                committed = self.plan is not None and remaining < P.t_commit
                outer = outer_tick or self.plan is None
                new = self.selector.update(
                    t, xR[XY], self.u_prev[0:2], self.puck, outer=outer,
                    commit_key=self.plan.key if committed else None,
                )
                if new is not None:
                    if self.verbose and (self.plan is None or new.key != self.plan.key):
                        print(f"t={t:.2f}s plan {new.label}: tf={new.tf:.3f}s cost={new.sol.cost:.3f} "
                              f"({'cold' if new.sol.cold else 'warm'}, {new.sol.iters} it, "
                              f"{1e3 * new.sol.t_wall:.1f} ms)")
                    self.plan = new
                    self.mode = ATT
                    self.plan_history.append((t, new.label, new.tf))

        if self.mode == ATT and self.plan is not None:
            # Close to impact the tracker grid collapses: finish with the plan's own input
            if self.tracker is not None and self.plan.remaining(t) > self.tracker.t_min:
                u = self.tracker.solve(self.plan, t, xR)
            else:
                u = np.append(self.plan.input_at(t), 0.0)  # a_theta = 0: the plan has no heading
            F = accel_planner_to_force_world(u, P)
        else:
            F = np.asarray(self.def_controller(jnp.asarray(state)), dtype=float)

        F = np.clip(F, -self.F_max, self.F_max)
        self.u_prev = force_world_to_accel_planner(F, P)
        return F

    ########################
    ### Logging helpers  ###
    ########################
    def plan_world(self, state):
        """Current plan as (N, 9) rows (x_{k+1} world, F_k world), the path_plans layout.
        The plan has no heading: its rows keep the current heading and rate, and zero torque."""
        P = self.P
        N = P.N
        rows = np.zeros((N, 9))
        rows[:, :6] = np.asarray(state[0:6], dtype=float)
        if self.mode != ATT or self.plan is None:
            return rows
        sol = self.plan.sol
        X = np.tile(state_world_to_planner(rows[0, :6], P), (N, 1))
        X[:, XY] = sol.X[:, 1:].T
        U = np.zeros((N, 3))
        U[:, 0:2] = sol.U.T
        return np.hstack([state_planner_to_world(X, P), accel_planner_to_force_world(U, P)])

    def candidates_world(self, n_shot=8):
        """
        Latest solution of every bounce sequence, selected or not:
        robot path (n, N+1, 2) and post-impact shot polyline (n, n_shot, 2) in world
        coordinates (NaN when not solved or infeasible), selection score, validity and
        the index of the executed sequence (-1 if none). Sequences not re-solved at an
        inner update keep their result from the last outer update.
        """
        P = self.P
        seqs = self.selector.seqs
        robot = np.full((len(seqs), P.N + 1, 2), np.nan)
        shot = np.full((len(seqs), n_shot, 2), np.nan)
        score = np.full(len(seqs), np.nan)
        valid = np.zeros(len(seqs), dtype=bool)
        chosen = -1
        for j, (s, k) in enumerate(seqs):
            plan = self.selector.candidates.get((s, k))
            if plan is None:
                continue
            sol = plan.sol
            score[j], valid[j] = plan.score, sol.valid
            if not sol.valid:
                continue
            robot[j] = pos_planner_to_world(sol.X[0:2].T, P)
            shot[j] = pos_planner_to_world(_pad_rows(shot_polyline(sol.p_c, sol.vp, P), n_shot), P)
            if self.mode == ATT and self.plan is not None and plan.key == self.plan.key:
                chosen = j
        return robot, shot, score, valid, chosen

    def predicted_puck_world(self, state, t, n_samples=60, horizon=3.0):
        """
        Predicted puck path in world coordinates, recomputed from the current puck state
        (display only, so it refreshes every simulation step, also after a contact).
        With a plan: free flight up to the planned impact, then the planned shot.
        Without: free flight with wall bounces until a goal or the horizon.
        Exact polyline vertices (wall hits), padded to n_samples rows with the end point.
        """
        P = self.P
        puck = state_world_to_planner(np.asarray(state[12:18], dtype=float), P)
        remaining = self.plan.remaining(t) if self.plan is not None else -1.0
        if self.mode == ATT and remaining > 0:
            sol = self.plan.sol
            path = np.vstack([bounce_path(puck[0:2], puck[3:5], P, remaining)[0], shot_polyline(sol.p_c, sol.vp, P)])
        else:
            path = bounce_path(puck[0:2], puck[3:5], P, horizon)[0]
        return pos_planner_to_world(_pad_rows(path, n_samples), P)


def _pad_rows(points, n):
    """Fixed-size copy of a polyline for the recorded arrays: repeat the end point."""
    points = np.asarray(points, dtype=float)
    if len(points) >= n:
        return np.vstack([points[: n - 1], points[-1:]])
    return np.vstack([points, np.repeat(points[-1:], n - len(points), axis=0)])
