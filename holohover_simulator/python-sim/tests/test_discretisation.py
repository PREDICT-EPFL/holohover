"""Test 1: exact ZOH map against the simulator, and the frame/unit conversions."""

import numpy as np
from scipy.linalg import expm

from conftest import DT, R2_PARK, PUCK_PARK, rollout
from controllers.shot_mpc.params import (
    XY, state_world_to_planner, state_planner_to_world, pos_world_to_planner,
    accel_planner_to_force_world, force_world_to_accel_planner,
)
from controllers.shot_mpc.model import discrete_matrices, zoh_coefficients


def test_zoh_matches_simulator(physics, P):
    r1 = [1.3, 0.5, 0.0, 0.2, 0.1, 0.0]
    F = np.array([0.3, -0.2, 0.0])
    steps = 50
    states, *_ = rollout(physics, np.array(r1 + R2_PARK + PUCK_PARK), steps, r1_force=F)

    x0 = state_world_to_planner(np.array(r1), P)[XY]
    u = force_world_to_accel_planner(F, P)[0:2]
    A, B = discrete_matrices(steps * DT, P.gamma)
    x_pred = A @ x0 + B @ u
    x_sim = state_world_to_planner(states[-1, :6], P)[XY]

    # The simulator integrates positions with explicit Euler: error ~ a T dt / 2 ~ 7 mm
    assert np.max(np.abs(x_pred[0:2] - x_sim[0:2])) < 0.015
    assert np.max(np.abs(x_pred[2:4] - x_sim[2:4])) < 0.01


def test_zoh_semigroup_and_gamma_limit(P):
    A1, B1 = discrete_matrices(0.5, P.gamma)
    Ah, Bh = discrete_matrices(0.5 / 50, P.gamma)
    A, B = np.eye(4), np.zeros((4, 2))
    for _ in range(50):  # constant input: composing 50 small steps must equal one big step
        A, B = Ah @ A, Ah @ B + Bh
    assert np.allclose(A, A1, atol=1e-12) and np.allclose(B, B1, atol=1e-12)

    dt = 0.05
    A0, B0 = discrete_matrices(dt, 0.0)
    I2 = np.eye(2)
    assert np.allclose(A0, np.block([[I2, dt * I2], [0 * I2, I2]]))
    assert np.allclose(B0, np.vstack([0.5 * dt**2 * I2, dt * I2]))

    # Smallest friction in use (gamma is 0 or >= 1e-3): the closed form of b, which cancels
    # for small gamma dt, still matches the matrix exponential of one axis' (p, v, u)
    g = 1e-3
    E = expm(np.array([[0.0, 1.0, 0.0], [0.0, -g, 1.0], [0.0, 0.0, 0.0]]) * dt)
    e1, a, b = zoh_coefficients(dt, g)
    assert np.allclose([e1, a, b], [E[1, 1], E[0, 1], E[0, 2]], rtol=1e-6, atol=0)


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
