"""Plan controller: the tracking QP reproduces a consistent plan and corrects the offsets the open-loop
plan keeps; the modes follow the plans, their expiry and the defence PD."""

import math

import numpy as np
import pytest

from holohover_control.plan import Plan, discrete_matrices
from holohover_control.plan_controller import ATT, DEF, PD, PlanController
from holohover_control.tracker import XY, Tracker

# python-sim's robot (0.35 N, 0.005 Nm on 0.116 kg, radius 0.07 m) and tracker weights
GAMMA = 0.3
ACC_MAX = np.array([0.35 / 0.116, 0.35 / 0.116, 0.005 / (0.5 * 0.116 * 0.07**2)])
WEIGHTS = dict(Q=[1e4, 1e4, 1e2, 1e2, 1e2, 1.0], Q_f=[1e6, 1e6, 1e2, 1e4, 1e4, 1.0], R=[1.0, 1.0, 1.0])
N, T0, TF = 20, 1.0, 0.5


@pytest.fixture(scope="module")
def plan():
    """Dynamically consistent plan (x, y, vx, vy), inputs at half the limits."""
    rng = np.random.default_rng(0)
    U = np.clip(np.cumsum(rng.normal(size=(N, 2)), axis=0), -1, 1) * 0.5 * ACC_MAX[:2]
    A, B = discrete_matrices(TF / N, GAMMA)
    X = np.zeros((N + 1, 4))
    X[0] = [0.3, 0.1, 0.5, -0.2]
    for k in range(N):
        X[k + 1] = A @ X[k] + B @ U[k]
    return Plan(label="direct", t0=T0, tf=TF, X=X, U=U)


def tracker(H=10):
    return Tracker(GAMMA, ACC_MAX, H, theta_ref=0.0, **WEIGHTS)


def state(x_plan, theta=0.0):
    """(x, y, theta, vx, vy, omega) from a plan state, with heading theta at rest."""
    x = np.zeros(6)
    x[XY] = x_plan
    x[2] = theta
    return x


def test_reference_hits_plan_nodes(plan):
    nodes = T0 + np.linspace(0.0, TF, N + 1)
    assert np.allclose(plan.state_at(nodes, GAMMA), plan.X.T, atol=1e-12)


def test_consistent_plan_is_reproduced(plan):
    # Tracker grid = plan grid and zero error: the optimum is the plan's own input sequence
    track = tracker(H=N).solve(plan, T0, state(plan.X[0]))
    U = np.vstack([plan.U.T, np.zeros(N)])
    assert np.allclose(track.U, U, atol=1e-6 * ACC_MAX.max())
    assert np.allclose(track.input_at(T0 + (np.arange(N) + 0.5) * TF / N), U)


def test_offset_is_corrected(plan):
    trk = tracker()
    dt = 0.01
    A, B = discrete_matrices(dt, GAMMA, axes=3)
    offset = np.array([0.02, -0.02, 0.2, 0.0, 0.0, 0.0])

    def run(tracking):
        ctrl = PlanController(trk if tracking else None, PD([0, 0, 0], [0] * 3, [0] * 3), ACC_MAX, 0.1)
        ctrl.on_plan(plan, T0)
        x = state(plan.X[0]) + offset
        for i in range(int(round(TF / dt))):
            x = A @ x + B @ ctrl.step(T0 + i * dt, x)
        return np.linalg.norm(x[0:2] - plan.X[-1, 0:2]), np.linalg.norm(x[3:5] - plan.X[-1, 2:4]), abs(x[2])

    pos_open, _, theta_open = run(tracking=False)
    pos_err, vel_err, theta_err = run(tracking=True)
    assert pos_open > 0.02 and theta_open == pytest.approx(0.2)
    assert pos_err < 1e-3 and vel_err < 0.02 and theta_err < 0.02


def test_modes(plan):
    ctrl = PlanController(tracker(), PD([0.7, 0.0, 0.0], [1.0] * 3, [0.5] * 3), ACC_MAX, plan_expiry=0.1)
    x = state(plan.X[0])
    assert ctrl.mode == DEF
    ctrl.on_plan(plan, T0)
    assert ctrl.mode == ATT
    ctrl.step(T0 + TF + 0.05, x)  # past the impact: the last solve, open loop
    assert ctrl.mode == ATT
    ctrl.step(T0 + TF + 0.11, x)  # expired
    assert ctrl.mode == DEF and ctrl.plan is None
    ctrl.on_plan(plan, T0 + TF + 0.2)  # already expired: ignored
    assert ctrl.mode == DEF
    ctrl.on_plan(plan, T0)
    ctrl.on_plan(None, T0 + 0.1)  # no plan: the planner saw a contact
    assert ctrl.mode == DEF


def test_defence_pd_is_clipped_and_wraps_the_heading():
    pd = PD([0.7, 0.0, 0.0], [10.0, 10.0, 1.0], [1.0, 1.0, 1.0])
    acc = pd([0.6, 0.1, 2 * math.pi - 0.1, 0.2, 0.0, 0.0])
    assert np.allclose(acc, [10 * 0.1 - 0.2, -1.0, 0.1])
    ctrl = PlanController(None, PD([0.7, 0.0, 0.0], [1e3] * 3, [0.0] * 3), ACC_MAX, 0.1)
    assert np.allclose(ctrl.step(0.0, np.zeros(6)), [ACC_MAX[0], 0.0, 0.0])
