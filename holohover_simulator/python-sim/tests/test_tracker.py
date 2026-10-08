"""Tracking QP: it reproduces a consistent plan and corrects position and heading offsets the open-loop plan keeps."""

from types import SimpleNamespace

import numpy as np
import pytest

from controllers.shot_mpc.params import XY
from controllers.shot_mpc.model import discrete_matrices
from controllers.shot_mpc.shot_selector import Plan
from controllers.shot_mpc.tracker import Tracker

T0, TF = 1.0, 0.5


@pytest.fixture(scope="module")
def plan(P):
    """Dynamically consistent plan (x, y, vx, vy) on the planner grid, inputs at half the limits."""
    rng = np.random.default_rng(0)
    U = np.clip(np.cumsum(rng.normal(size=(2, P.N)), axis=1), -1, 1) * 0.5 * P.u_max[:2, None]
    A, B = discrete_matrices(TF / P.N, P.gamma)
    X = np.zeros((4, P.N + 1))
    X[:, 0] = [0.3, 0.1, 0.5, -0.2]
    for k in range(P.N):
        X[:, k + 1] = A @ X[:, k] + B @ U[:, k]
    return Plan(key=(0, 0), label="-", ybar=0.0, t0=T0, sol=SimpleNamespace(X=X, U=U, tf=TF))


def tracker_state(x_plan, theta):
    """Tracker state (x, y, theta, vx, vy, omega) from a plan state, with heading theta at rest."""
    x = np.zeros(6)
    x[XY] = x_plan
    x[2] = theta
    return x


def test_reference_hits_plan_nodes(P, plan):
    nodes = T0 + np.linspace(0.0, TF, P.N + 1)
    assert np.allclose(plan.state_at(nodes, P.gamma), plan.sol.X, atol=1e-12)


def test_consistent_plan_is_reproduced(config, P, plan):
    # Tracker grid = plan grid and zero error: the optimum is the plan's own input sequence
    tracker = Tracker(P, {**config["shot_mpc"]["tracker"], "H": P.N})
    track = tracker.solve(plan, T0, tracker_state(plan.sol.X[:, 0], tracker.theta_ref))
    U = np.vstack([plan.sol.U, np.zeros(P.N)])
    assert np.allclose(track.U, U, atol=1e-6 * P.u_max.max())
    assert np.allclose(track.input_at(T0 + (np.arange(P.N) + 0.5) * TF / P.N), U)


def test_offset_is_corrected(config, P, plan):
    tracker = Tracker(P, config["shot_mpc"]["tracker"])
    dt = 0.01
    A, B = discrete_matrices(dt, P.gamma, axes=3)
    offset = np.array([0.02, -0.02, 0.2, 0.0, 0.0, 0.0])

    def run(tracking):
        x = tracker_state(plan.sol.X[:, 0], tracker.theta_ref) + offset
        for i in range(int(round(TF / dt))):
            t = T0 + i * dt
            if not tracking:
                u = np.append(plan.input_at(t), 0.0)
            else:
                if plan.remaining(t) > 0:  # as ShotController: re-solve until the impact
                    track = tracker.solve(plan, t, x)
                u = track.input_at(t)
            x = A @ x + B @ u
        return (np.linalg.norm(x[0:2] - plan.sol.X[0:2, -1]), np.linalg.norm(x[3:5] - plan.sol.X[2:4, -1]),
                abs(x[2] - tracker.theta_ref))

    pos_open, _, theta_open = run(tracking=False)
    pos_err, vel_err, theta_err = run(tracking=True)
    assert pos_open > 0.02 and theta_open == pytest.approx(0.2)
    assert pos_err < 1e-3 and vel_err < 0.02 and theta_err < 0.02
