"""Tracking QP: it reproduces a consistent plan and corrects an offset the open-loop plan keeps."""

from types import SimpleNamespace

import numpy as np
import pytest

from shot_mpc.shot_nlp import discrete_matrices
from shot_mpc.shot_selector import Plan
from shot_mpc.tracker import Tracker, plan_reference

T0, TF = 1.0, 0.5


@pytest.fixture(scope="module")
def plan(P):
    """Dynamically consistent plan on the planner grid, inputs at half the limits."""
    rng = np.random.default_rng(0)
    U = np.clip(np.cumsum(rng.normal(size=(3, P.N)), axis=1), -1, 1) * 0.5 * P.u_max[:, None]
    A, B = discrete_matrices(TF / P.N, P.gamma)
    X = np.zeros((6, P.N + 1))
    X[:, 0] = [0.3, 0.1, np.pi, 0.5, -0.2, 0.0]
    for k in range(P.N):
        X[:, k + 1] = A @ X[:, k] + B @ U[:, k]
    return Plan(key=(0, 0, 0), label="-", ybar=0.0, t0=T0, sol=SimpleNamespace(X=X, U=U, tf=TF))


def test_reference_hits_plan_nodes(P, plan):
    nodes = plan.node_times()
    assert np.allclose(plan_reference(plan, nodes, P.gamma), plan.sol.X, atol=1e-12)


def test_consistent_plan_is_reproduced(config, P, plan):
    # Tracker grid = plan grid and zero error: the optimum is the plan's own input
    tracker = Tracker(P, {**config["shot_mpc"]["tracker"], "H": P.N})
    u = tracker.solve(plan, T0, plan.sol.X[:, 0])
    assert np.allclose(u, plan.sol.U[:, 0], atol=1e-6 * P.u_max.max())


def test_offset_is_corrected(config, P, plan):
    tracker = Tracker(P, config["shot_mpc"]["tracker"])
    dt = 0.01
    A, B = discrete_matrices(dt, P.gamma)
    offset = np.array([0.02, -0.02, 0.0, 0.0, 0.0, 0.0])

    def run(track):
        x = plan.sol.X[:, 0] + offset
        for i in range(int(round(TF / dt))):
            t = T0 + i * dt
            u = tracker.solve(plan, t, x) if track and plan.remaining(t) > tracker.t_min else plan.input_at(t)
            x = A @ x + B @ u
        return np.linalg.norm(x[0:2] - plan.sol.X[0:2, -1]), np.linalg.norm(x[3:5] - plan.sol.X[3:5, -1])

    pos_open, _ = run(track=False)
    pos_err, vel_err = run(track=True)
    assert pos_open > 0.02
    assert pos_err < 1e-3 and vel_err < 0.02
