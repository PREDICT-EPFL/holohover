"""
Robot-puck contact detection from the planner's puck model.

The puck state (from its EKF) is compared with the free flight predicted from an earlier
sample, wall reflections included (model.flight_after), so wall bounces are explained by the
model. A contact is a velocity the model does not explain, with the robot next to the puck at
some sample of the look-back window: the EKF shows a velocity jump only a few samples after the
contact, when the puck may already have left the robot.
"""

from collections import deque

import numpy as np

from .shot_mpc.model import flight_after


class ContactDetector:
    def __init__(self, P, dv=0.3, margin=0.03, lookback=0.05):
        """dv: velocity error (m/s) that counts as a contact; margin: largest robot-puck gap (m)
        at a contact; lookback: age (s) of the sample the prediction starts from, so that the
        EKF's smoothing of a velocity jump over a few samples adds up."""
        self.P = P
        self.dv = dv
        self.reach = P.r_R + P.r_p + margin
        self.lookback = lookback
        self.samples = deque()  # (t, q, v, robot-puck distance), oldest first

    def reset(self):
        self.samples.clear()

    def update(self, t, q, v, robot):
        """Puck position q, velocity v at time t and robot position, planner frame. True on a contact."""
        q, v = np.asarray(q, dtype=float), np.asarray(v, dtype=float)
        if self.samples and t <= self.samples[-1][0]:
            return False  # not newer than the last sample
        # Start from the newest sample at least lookback old
        while len(self.samples) > 1 and t - self.samples[1][0] >= self.lookback:
            self.samples.popleft()
        ref = self.samples[0] if self.samples else None
        self.samples.append((t, q, v, np.linalg.norm(np.asarray(robot, dtype=float) - q)))
        if ref is None:
            return False
        t0, q0, v0, _ = ref
        _, v_pred = flight_after(q0, v0, self.P, t - t0)
        near = min(sample[3] for sample in self.samples) <= self.reach
        return bool(near and np.linalg.norm(v - v_pred) > self.dv)
