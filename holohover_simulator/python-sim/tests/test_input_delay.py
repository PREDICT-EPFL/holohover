"""simulation.input_delay: robot 1's input computed at step i is applied at step i + delay, and
shot_mpc's delay compensation."""

import copy

import numpy as np
import jax.numpy as jnp
import pytest

from controllers import ATT
from controllers.shot_mpc.params import (XY, force_world_to_accel_planner, state_planner_to_world,
                                         state_world_to_planner)
from controllers.shot_mpc.shot_controller import ShotController
from simulation.episode import Episode, n_substeps, physics_substeps
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


def test_compensation_predicts_plant(config, physics, P, nlp):
    """ShotController.compensate puts robot 1 and the puck where the simulator takes them under the
    pending inputs, the puck across a wall hit."""
    d = 4
    config = copy.deepcopy(config)
    config["shot_mpc"]["delay_compensation"] = d
    ctrl = ShotController(config, physics, P, nlp, verbose=False)
    forces = np.random.default_rng(0).uniform(-1, 1, (d, 3)) * config["simulation"]["u_limits"]
    forces[:, 2] = 0.0  # the simulator applies no torque
    for F in forces:
        ctrl.pending.append(force_world_to_accel_planner(F, P))

    state = initial_state(config)
    state[0:6] = [1.7, 0.6, 0.0, 0.3, -0.2, 0.0]
    state[12:18] = [1.2, 0.06, 0.0, 0.5, -1.0, 0.0]  # hits the bottom wall after ~25 ms
    n = n_substeps(config)
    s = jnp.asarray(state, dtype=jnp.float32)
    for F in forces:
        s, *_ = physics_substeps(s, jnp.asarray(np.concatenate([F, np.zeros(3)]), dtype=jnp.float32), physics,
                                 0.01 / n, n)
    s = np.asarray(s, dtype=float)

    xR, puck = ctrl.compensate(state_world_to_planner(state[0:6], P), state_world_to_planner(state[12:18], P))
    assert puck[4] * state_world_to_planner(state[12:18], P)[4] < 0  # the prediction bounced
    np.testing.assert_allclose(xR[XY], state_world_to_planner(s[0:6], P)[XY], atol=1e-4)
    np.testing.assert_allclose(puck[XY], state_world_to_planner(s[12:18], P)[XY], atol=1e-4)


def test_compensated_defence(config, physics, P, nlp):
    """In defence, the PD acts on the predicted robot state."""
    d = 3
    config = copy.deepcopy(config)
    config["shot_mpc"]["delay_compensation"] = d
    ctrl = ShotController(config, physics, P, nlp, verbose=False)
    ctrl.attack_after = np.inf  # no planning: defence
    for _ in range(d):
        ctrl.pending.append(force_world_to_accel_planner([-0.3, 0.2, 0.0], P))
    state = initial_state(config)
    state[0:6] = [1.7, 0.6, 0.0, 0.3, -0.2, 0.0]

    predicted = state.copy()
    predicted[0:6] = state_planner_to_world(ctrl.compensate(state_world_to_planner(state[0:6], P),
                                                            state_world_to_planner(state[12:18], P))[0], P)
    F = ctrl.step(0, state)
    np.testing.assert_allclose(F, np.clip(ctrl.def_controller(jnp.asarray(predicted)), -ctrl.F_max, ctrl.F_max),
                               rtol=1e-6)
    assert not np.allclose(F, np.clip(ctrl.def_controller(jnp.asarray(state)), -ctrl.F_max, ctrl.F_max))
