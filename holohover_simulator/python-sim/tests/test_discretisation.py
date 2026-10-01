"""Test 1: exact ZOH map against the simulator, and the frame/unit conversions."""

import numpy as np

from conftest import DT, R2_PARK, PUCK_PARK, rollout
from shot_mpc.params import (
    state_world_to_planner, state_planner_to_world, pos_world_to_planner,
    accel_planner_to_force_world, force_world_to_accel_planner,
)
from shot_mpc.shot_nlp import discrete_matrices


def test_zoh_matches_simulator(physics, P):
    r1 = [1.3, 0.5, 0.0, 0.2, 0.1, 0.0]
    F = np.array([0.3, -0.2, 0.0])
    steps = 50
    states, *_ = rollout(physics, np.array(r1 + R2_PARK + PUCK_PARK), steps, r1_force=F)

    x0 = state_world_to_planner(np.array(r1), P)
    u = force_world_to_accel_planner(F, P)
    A, B = discrete_matrices(steps * DT, P.gamma)
    x_pred = A @ x0 + B @ u
    x_sim = state_world_to_planner(states[-1, :6], P)

    # The simulator integrates positions with explicit Euler: error ~ a T dt / 2 ~ 7 mm
    assert np.max(np.abs(x_pred[0:2] - x_sim[0:2])) < 0.015
    assert np.max(np.abs(x_pred[3:5] - x_sim[3:5])) < 0.01


def test_zoh_semigroup_and_gamma_limit(P):
    A1, B1 = discrete_matrices(0.5, P.gamma)
    Ah, Bh = discrete_matrices(0.5 / 50, P.gamma)
    A, B = np.eye(6), np.zeros((6, 3))
    for _ in range(50):  # constant input: composing 50 small steps must equal one big step
        A, B = Ah @ A, Ah @ B + Bh
    assert np.allclose(A, A1, atol=1e-12) and np.allclose(B, B1, atol=1e-12)

    dt = 0.05
    A0, B0 = discrete_matrices(dt, 0.0)
    I3 = np.eye(3)
    assert np.allclose(A0, np.block([[I3, dt * I3], [0 * I3, I3]]))
    assert np.allclose(B0, np.vstack([0.5 * dt**2 * I3, dt * I3]))
    As, Bs = discrete_matrices(dt, 1e-7)
    assert np.allclose(As, A0, atol=1e-8) and np.allclose(Bs, B0, atol=1e-8)


def test_frame_transforms(P):
    rng = np.random.default_rng(0)
    s = rng.normal(size=(5, 6))
    assert np.allclose(state_planner_to_world(state_world_to_planner(s, P), P), s)
    F = rng.normal(size=(5, 3))
    assert np.allclose(accel_planner_to_force_world(force_world_to_accel_planner(F, P), P), F)

    # Opponent goal (world x = 0) is at planner x = X, table centre line y = H/2 is y_p = 0,
    # and wall L (+y_p) is the world bottom wall
    assert np.allclose(pos_world_to_planner([0.0, 0.5 * P.W], P), [P.X, 0.0])
    assert pos_world_to_planner([1.0, 0.0], P)[1] == 0.5 * P.W
