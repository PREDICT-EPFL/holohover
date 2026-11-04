"""Actuation: no command gives no acceleration (not even the disturbance compensation); a constant
acceleration is reached once the motors have spun up, and the disturbance estimate is compensated.
Needs holohover_common's bindings (ROS workspace): skipped outside it."""

from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("holohover_common.bindings")
from holohover_control.actuation import Actuation  # noqa: E402

PROPS = Path(__file__).resolve().parents[2] / "holohover_utils/config/common/holohover_params_new.yaml"
PERIOD = 0.01
STATE = np.array([0.2, -0.1, 0.0, 0.0, 0.3, 0.0])  # x, y, v_x, v_y, yaw, w_z: at rest, turned


def achieved(act, state):
    """Acceleration the motors give now."""
    m = act.model
    return m.control_force_to_acceleration(state, m.signal_to_thrust(act.motor_velocities))


def test_no_command_is_no_acceleration():
    act = Actuation(str(PROPS), PERIOD)
    for _ in range(50):
        act.step(STATE, np.array([0.5, 0.5, 0.0]), None)  # a disturbance, but no command
    np.testing.assert_allclose(achieved(act, STATE), 0.0, atol=1e-3)


@pytest.mark.parametrize("disturbance", [np.zeros(3), np.array([0.1, -0.05, 0.0])])
def test_constant_acceleration_is_reached(disturbance):
    act = Actuation(str(PROPS), PERIOD)
    acc = np.array([0.6, -0.3, 0.0])
    for _ in range(100):  # 1 s: the motor lag is ~0.03 s
        act.step(STATE, disturbance, acc)
    np.testing.assert_allclose(achieved(act, STATE), acc - disturbance, atol=1e-3)
