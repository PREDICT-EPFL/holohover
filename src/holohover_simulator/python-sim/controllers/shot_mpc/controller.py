"""
Robot 1's shot MPC in python-sim: the planner and the controller of the ROS nodes, called in turn at
every control step.

    planner     holohover_planner ShotPlanner (the shot_planner node's)
    controller  holohover_control PlanController and Tracker (the control_plan node's)

At each control step (simulation.hz):
- every 1 / inner_hz the planner ticks from the true states, and its plans go straight to the
  controller: a plan starts and arrives at the tick's time (no solve latency);
- the controller's acceleration becomes robot 1's input F = (m a_x, m a_y, I a_theta), clipped.
Contacts are the simulator's (notify_contact). The planner's last input (u_prev) is the applied one.
"""

import numpy as np

from holohover_control.plan import Plan
from holohover_control.plan_controller import PD, PlanController
from holohover_control.tracker import Tracker
from holohover_planner.shot_mpc.model import bounce_path
from holohover_planner.shot_mpc.params import Frame
from holohover_planner.shot_mpc.report import timing_summary, update_json
from holohover_planner.shot_mpc.shot_nlp import ShotNLP
from holohover_planner.shot_planner import Body, ShotPlanner

from controllers import ATT

from .frames import (XY, body_mass, controller_parameters, planner_parameters, pos_planner_to_world, pos_table_to_world,
                     shot_params, state_world_to_planner, state_world_to_table)

N_FUTURE = 60  # samples of the recorded predicted puck path


def make_tracker(config):
    """The tracking QP (control_plan_config.yaml), or None when python-sim runs the plans open loop
    (main.py --no-tracker: config shot_mpc.tracker false). Building it takes ~14 ms and it keeps no episode
    state: build it once and pass it to every ShotController."""
    if not config.get("shot_mpc", {}).get("tracker", True):
        return None
    c = controller_parameters()
    return Tracker(c["tracker.friction"], c["acc_max"], c["tracker.H"], c["tracker.yaw_ref"], c["tracker.Q"],
                   c["tracker.Q_f"], c["tracker.R"], 1.0 / config["simulation"]["hz"])


def plan_controller(tracker):
    """Robot 1's controller: the tracker, and the defence PD, acceleration bounds and plan expiry of
    control_plan_config.yaml."""
    c = controller_parameters()
    return PlanController(tracker, PD(c["defence.home"], c["defence.kp"], c["defence.kd"]), c["acc_max"],
                          c["plan_expiry"])


class ShotFactory:
    """What every episode shares, built once per process: the NLP (compiled), the worker processes
    of its outer updates (shot_mpc.n_workers of shot_planner_config.yaml, one per bounce sequence) and the
    tracker."""

    def __init__(self, config, workers=None):
        self.config = config
        self.P = shot_params(config)
        self.nlp = ShotNLP(self.P)
        n_workers = planner_parameters(config)["shot_mpc.n_workers"] if workers is None else workers
        pool = self.nlp.start_pool(n_workers)
        if pool is not None:
            print(f"[solve_pool] {pool.n_workers} worker processes for the outer updates")
        self.tracker = make_tracker(config)

    def new(self, physics, init_state, verbose=False):
        return ShotController(self.config, self.P, self.nlp, self.tracker, verbose=verbose)

    def summary(self, logs):
        """Solve times over the episode logs (ShotController.log)."""
        workers = self.nlp.pool.n_workers if self.nlp.pool else 0
        lines = timing_summary([u for log in logs for u in log["updates"]]).split("\n")
        tracker = np.array([t for log in logs for t in log["tracker_times"]]) * 1e3
        if len(tracker):
            lines.insert(5, f"  {'tracker (ref + QP)':<22} n={len(tracker):<5d} mean={tracker.mean():7.2f}  "
                            f"p95={np.percentile(tracker, 95):7.2f}  max={tracker.max():7.2f} ms")
        return f"tracker: {self.tracker is not None} | workers: {workers}\n" + "\n".join(lines)

    def close(self):
        if self.nlp.pool is not None:
            self.nlp.pool.close()


class ShotController:
    def __init__(self, config, P, nlp, tracker, verbose=True):
        self.P = P
        log = print if verbose else None
        self.planner = ShotPlanner(P, Frame(P, "left"), nlp, log=log)
        self.controller = plan_controller(tracker)
        self.candidate_labels = self.planner.labels

        hz = config["simulation"]["hz"]
        self.dt = 1.0 / hz
        self.inner_every = max(1, int(round(hz / P.inner_hz)))
        self.F_max = np.array(config["simulation"]["u_limits"], dtype=float)
        self.mass = body_mass(P)
        self.u_prev = np.zeros(3)  # last applied input as an acceleration, table frame
        self.plan_history = []  # (t, label, tf) of every adopted plan
        self.plans = None  # the planner's PlanSet of the last step (None: no update), recorded for /h0/plan
        self.contact_plans = None  # the PlanSet of a contact since the last step

    @property
    def mode(self):
        return self.controller.mode

    def notify_contact(self, t):
        """Robot 1 touched the puck: the planner drops its plan, the controller defends."""
        plans = self.planner.on_contact(t)
        if plans is not None:
            self.controller.on_plan(None, t)
            self.contact_plans = plans  # recorded with the next step (contacts come between two steps)

    def step(self, i, state):
        """Return the world-frame input (Fx, Fy, tau) of robot 1 at simulation step i."""
        t = i * self.dt
        state = np.asarray(state, dtype=float)
        robot = state_world_to_table(state[0:6], self.P)
        self.plans, self.contact_plans = self.contact_plans, None
        if i % self.inner_every == 0:
            puck = state_world_to_table(state[12:18], self.P)
            plans = self.planner.tick(t, Body(t, *robot[XY]), Body(t, *puck[XY]), u_prev=self.u_prev[0:2])
            if plans is not None:
                self.plans = plans
                c = plans.plan
                self.controller.on_plan(None if c is None else Plan.from_candidate(c), t)
            if self.planner.adopted is not None:
                self.plan_history.append((t, self.planner.adopted.label, self.planner.adopted.tf))
        F = np.clip(self.controller.step(t, robot) * self.mass, -self.F_max, self.F_max)
        self.u_prev = F / self.mass
        return F

    ########################
    ### Logging helpers  ###
    ########################
    def record(self, state, t):
        """What the viewers show at this step (simulation.episode.episode_data)."""
        future = np.zeros((N_FUTURE, 18))
        future[:, 6:9] = state[6:9]
        future[:, 12:14] = self.predicted_puck_world(state, t, N_FUTURE)
        plan = self.controller.plan
        row = {
            "plan_input_sequence": self.controller.acc_plan * self.mass,
            "def_input_sequence": self.controller.acc_def * self.mass,
            "path_plans": self.plan_world(state),
            "tfs": max(plan.remaining(t), 0.0) if self.mode == ATT and plan else 0.0,
            "future_trajs": future,
            "ros_plan": self.plans,
        }
        row.update(zip(("cand_robot", "cand_shot", "cand_score", "cand_valid", "cand_chosen"),
                       self.candidates_world()))
        return row

    def log(self):
        """Episode log: adopted plans (t, label, tf), selector updates and tracker solve times."""
        return {"plan_history": self.plan_history, "updates": [update_json(u) for u in self.planner.selector.logs],
                "tracker_times": self.controller.tracker_times}

    def plan_world(self, state):
        """Current plan as (N, 9) rows (x_{k+1} world, F_k world), the path_plans layout.
        The plan has no heading: its rows keep the current heading and rate, and zero torque."""
        N = self.P.N
        rows = np.zeros((N, 9))
        rows[:, :6] = np.asarray(state[0:6], dtype=float)
        plan = self.controller.plan
        if self.mode != ATT or plan is None:
            return rows
        rows[:, 0:2] = pos_table_to_world(plan.X[1:, 0:2], self.P)
        rows[:, 3:5] = plan.X[1:, 2:4]
        rows[:, 6:8] = plan.U * self.P.m_R
        return rows

    def candidates_world(self, n_shot=8):
        """
        Latest solution of every bounce sequence, selected or not:
        robot path (n, N+1, 2) and post-impact shot polyline (n, n_shot, 2) in world
        coordinates (NaN when not solved or infeasible), cost J*(B) (the selection criterion), validity and
        the index of the executed sequence (-1 if none). Sequences not re-solved at an
        inner update keep their result from the last outer update.
        """
        P = self.P
        selector, followed = self.planner.selector, self.planner.plan
        seqs = selector.seqs
        robot = np.full((len(seqs), P.N + 1, 2), np.nan)
        shot = np.full((len(seqs), n_shot, 2), np.nan)
        score = np.full(len(seqs), np.nan)
        valid = np.zeros(len(seqs), dtype=bool)
        chosen = -1
        for j, (s, k) in enumerate(seqs):
            plan = selector.candidates.get((s, k))
            if plan is None:
                continue
            sol = plan.sol
            score[j], valid[j] = sol.cost, sol.valid
            if not sol.valid:
                continue
            robot[j] = pos_planner_to_world(sol.X[0:2].T, P)
            shot[j] = pos_planner_to_world(_pad_rows(_shot_path(sol, P), n_shot), P)
            if self.mode == ATT and followed is not None and plan.key == followed.key:
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
        plan = self.planner.plan
        remaining = plan.remaining(t) if plan is not None else -1.0
        if self.mode == ATT and remaining > 0:
            path = np.vstack([bounce_path(puck[0:2], puck[3:5], P, remaining)[0], _shot_path(plan.sol, P)])
        else:
            path = bounce_path(puck[0:2], puck[3:5], P, horizon)[0]
        return pos_planner_to_world(_pad_rows(path, n_samples), P)


def _shot_path(sol, P):
    """Planned shot: vertices of the free puck path from the impact to the goal (at T_B)."""
    return bounce_path(sol.p_c, sol.vp, P, sol.terms["T_B"])[0]


def _pad_rows(points, n):
    """Fixed-size copy of a polyline for the recorded arrays: repeat the end point."""
    points = np.asarray(points, dtype=float)
    if len(points) >= n:
        return np.vstack([points[: n - 1], points[-1:]])
    return np.vstack([points, np.repeat(points[-1:], n - len(points), axis=0)])
