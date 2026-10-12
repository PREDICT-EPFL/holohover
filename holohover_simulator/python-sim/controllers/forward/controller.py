"""
Forward controller: a constant force forward.force (N) on robot 1 along -x (toward the left
goal), no torque. It has no plan and no prediction: it records the current state in their place.
"""

import numpy as np

from controllers import ATT


class ForwardFactory:
    def __init__(self, config):
        self.config = config

    def new(self, physics, init_state, verbose=False):
        return ForwardController(self.config)

    def summary(self, logs):
        return ""

    def close(self):
        pass


class ForwardController:
    def __init__(self, config):
        u_max = np.array(config["simulation"]["u_limits"])
        self.u = np.clip([-config["forward"]["force"], 0.0, 0.0], -u_max, u_max)
        self.mode = ATT

    def step(self, i, state):
        """Robot 1's input at control step i (world frame, clipped)."""
        return self.u.copy()

    def notify_contact(self, t):
        pass

    def record(self, state, t):
        state = np.asarray(state, dtype=float)
        return {"path_plans": np.concatenate([state[:6], self.u])[None],  # (1, 9)
                "tfs": np.nan,  # no planned impact
                "future_trajs": state[None]}  # (1, 18)

    def log(self):
        return {}
