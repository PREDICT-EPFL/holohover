"""
The plan controller without ROS: the ROS node (control_plan_node.py) and python-sim
(holohover_simulator/python-sim) both run this class. Table frame, times in s, outputs are
accelerations (a_x, a_y, a_theta); turning them into forces or motor signals is the caller's job.

    controller = PlanController(tracker, PD(home, kp, kd), acc_max, plan_expiry)
    controller.on_plan(plan, t)     # a planner's plan (plan.Plan), or None: no plan
    acc = controller.step(t, x, t_x)  # every control step, x = (x, y, theta, vx, vy, omega) measured at t_x

- DEF -> ATT as soon as a plan arrives (planners only send valid ones). Each new plan replaces the
  current one.
- ATT -> DEF on a message with no plan (the planner saw a contact or dropped its plan), or when
  the planned impact has passed by plan_expiry without a new plan.
- In ATT the tracker is re-solved at every step until the impact; past the impact its last solve
  is replayed open loop. Without a tracker, the plan's ZOH input is applied.
- The tracker solves from the state measured at t_x advanced to t with the inputs already sent (each
  acting from its step's time), with its own model.
- In DEF a PD holds the hovercraft at home.
"""

import bisect
import math
import time
from collections import deque

import numpy as np

from .plan import discrete_matrices

DEF, ATT = 0, 1  # defend at home, attack the puck


class PD:
    """a = kp (home - p) - kd v on (x, y, theta), the heading error wrapped to [-pi, pi]."""

    def __init__(self, home, kp, kd):
        self.home, self.kp, self.kd = (np.array(v, dtype=float) for v in (home, kp, kd))

    def __call__(self, x):
        x = np.asarray(x, dtype=float)
        e = self.home - x[0:3]
        e[2] = math.remainder(e[2], 2.0 * math.pi)
        return self.kp * e + self.kd * (0.0 - x[3:6])


class PlanController:
    def __init__(self, tracker, defence, acc_max, plan_expiry, log=None):
        """tracker: tracker.Tracker, or None to apply the plan's inputs open loop. defence: state ->
        acceleration (PD). acc_max: bounds of the output (a_x, a_y, a_theta). plan_expiry: s past the
        impact without a new plan, back to defence. log: callable(str) for the mode switches."""
        self.tracker, self.defence = tracker, defence
        self.acc_max = np.array(acc_max, dtype=float)
        self.plan_expiry = plan_expiry
        self.log = log or (lambda message: None)

        self.mode = DEF
        self.plan = None  # the plan being followed (plan.Plan)
        self.sent = deque(maxlen=50)  # (t, input) of the last steps, for the state age prediction
        self.track = None  # the last tracker solve on it (tracker.TrackerInputs)
        # The plan's and the PD's accelerations at the last step, before clipping (zero while the
        # other one is applied), and the wall time of every tracker solve
        self.acc_plan = np.zeros(3)
        self.acc_def = np.zeros(3)
        self.tracker_times = []

    def set_mode(self, mode, reason):
        if mode != self.mode:
            self.log(f"{'attack' if mode == ATT else 'defence'}: {reason}")
        self.mode = mode
        if mode == DEF:
            self.plan = self.track = None

    def reset(self):
        """A new episode: defence, no plan."""
        self.sent.clear()
        self.set_mode(DEF, "reset")

    def on_plan(self, plan, t):
        """A planner's message at time t: the plan to follow, or None."""
        if plan is None:
            self.set_mode(DEF, "no plan")
        elif plan.remaining(t) < -self.plan_expiry:
            self.set_mode(DEF, f"plan {plan.label} already expired")
        else:
            self.plan, self.track = plan, None
            self.set_mode(ATT, f"plan {plan.label}")

    def predict(self, t, x, t_x):
        """The state at t from x measured at t_x, under the inputs sent (each one from its step's time
        until the next one's), with the tracker's model (x itself if t <= t_x)."""
        starts = [t_j for t_j, _ in self.sent]
        s = t_x
        while t - s > 1e-12:
            i = bisect.bisect_right(starts, s) - 1  # the input acting at s
            u = self.sent[i][1] if i >= 0 else np.zeros(3)
            end = min(starts[i + 1], t) if i + 1 < len(starts) else t
            A_d, B_d = discrete_matrices(end - s, self.tracker.gamma, axes=3)
            x = A_d @ x + B_d @ u
            s = end
        return x

    def step(self, t, x, t_x=None):
        """The acceleration (a_x, a_y, a_theta) at time t from the state x measured at t_x (default t),
        clipped to acc_max."""
        x = np.asarray(x, dtype=float)
        if self.mode == ATT and self.plan.remaining(t) < -self.plan_expiry:
            self.set_mode(DEF, f"plan {self.plan.label} expired without a new one")
        self.acc_plan = np.zeros(3)
        self.acc_def = np.zeros(3)
        if self.mode == ATT:
            self.acc_plan = np.append(self.plan.input_at(t), 0.0)  # a_theta = 0: the plan has no heading
            if self.tracker is None:
                u = self.acc_plan
            else:
                # Re-solve until the impact; past it the last solve runs open loop
                if self.plan.remaining(t) > 0:
                    t_start = time.perf_counter()
                    self.track = self.tracker.solve(self.plan, t, self.predict(t, x, t if t_x is None else t_x))
                    self.tracker_times.append(time.perf_counter() - t_start)
                u = self.acc_plan if self.track is None else self.track.input_at(t)
        else:
            u = self.acc_def = self.defence(x)
        u = np.clip(u, -self.acc_max, self.acc_max)
        self.sent.append((t, u))
        return u
