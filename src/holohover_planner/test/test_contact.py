import numpy as np

from holohover_planner.contact import ContactDetector
from holohover_planner.shot_mpc.model import flight_after

DT = 0.01


def _run(P, q0, v0, robot, steps, kick_at=None, kick=(0.0, 0.0)):
    """Puck samples along the model's flight (a velocity kick at step kick_at); True if a contact fired."""
    det = ContactDetector(P)
    q, v = np.array(q0, float), np.array(v0, float)
    fired = False
    for i in range(steps):
        if i == kick_at:
            v = v + kick
        fired |= det.update(i * DT, q, v, robot)
        q, v = flight_after(q, v, P, DT)
    return fired


def test_free_flight_with_wall_bounce_is_no_contact(P):
    # Puck heading into wall L next to the robot: the bounce is in the model
    assert not _run(P, [0.8, P.w - 0.05], [0.3, 1.0], robot=[0.8, P.w - 0.1], steps=30)


def test_kick_next_to_robot_is_contact(P):
    assert _run(P, [0.8, 0.0], [-0.2, 0.0], robot=[0.8 - 0.105, 0.0], steps=20, kick_at=10, kick=(1.0, 0.2))


def test_kick_far_from_robot_is_no_contact(P):
    assert not _run(P, [0.8, 0.0], [-0.2, 0.0], robot=[0.2, 0.3], steps=20, kick_at=10, kick=(1.0, 0.2))


def test_kick_seen_after_the_puck_left_the_robot_is_contact(P):
    """The EKF lags: the velocity jump shows up two samples after the contact, when the puck is
    already out of reach."""
    det = ContactDetector(P)
    robot = np.array([0.6, 0.0])
    q, v = np.array([0.6 + P.r_R + P.r_p, 0.0]), np.zeros(2)
    fired = False
    for i in range(10):
        if i >= 3:  # contact at step 3: the puck leaves at 2 m/s
            q = q + np.array([2.0 * DT, 0.0])
        if i >= 5:  # the EKF velocity catches up at step 5
            v = np.array([2.0, 0.0])
        fired |= det.update(i * DT, q, v, robot)
    assert fired
