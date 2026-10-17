"""simulation.input_delay: robot 1's input computed at step i is applied at step i + delay."""

import copy

import numpy as np
import pytest

from controllers import ATT
from simulation.episode import Episode
from simulation.setup import initial_state


class RampController:
    """A different input at every step: Fx = -1e-3 * (i + 1)."""
    mode = ATT

    def step(self, i, state):
        return np.array([-1e-3 * (i + 1), 0.0, 0.0])

    def notify_contact(self, t):
        pass

    def record(self, state, t):
        return {}

    def log(self):
        return {}


def run(config, physics, delay, steps=8):
    config = copy.deepcopy(config)
    config["simulation"]["input_delay"] = delay
    ep = Episode(config, physics, RampController(), initial_state(config))
    rows = [ep.step() for _ in range(steps)]
    return np.array([row["input_sequence"] for row in rows]), np.array([row["states"] for row in rows])


@pytest.mark.parametrize("delay", [1, 3])
def test_inputs_shifted(config, physics, delay):
    applied, _ = run(config, physics, delay)
    commanded, _ = run(config, physics, 0)
    np.testing.assert_array_equal(applied[:delay], 0.0)
    np.testing.assert_array_equal(applied[delay:], commanded[:-delay])


def test_robot_waits(config, physics):
    """Robot 1 starts at rest and doesn't move before the first input arrives."""
    _, states = run(config, physics, 3)
    np.testing.assert_allclose(states[:3, 0:6], np.tile(initial_state(config)[0:6], (3, 1)), atol=1e-6)  # float32
    assert states[3, 3] < 0.0
