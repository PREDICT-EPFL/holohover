"""Test 6: derivatives of P(B), cold-start convergence, and a closed-loop smoke test."""

import numpy as np
import pytest

from controllers.shot_mpc.params import XY, state_world_to_planner
from controllers.shot_mpc.model import predict
from controllers.shot_mpc.shot_selector import ShotSelector


def _planner_setup(config, P, scenario):
    """Robot state (x, y, vx, vy) and puck flight of a scenario, planner frame."""
    sc = config["scenarios"][scenario]
    xR = state_world_to_planner(np.array(sc["robot1"], dtype=float), P)[XY]
    puck = state_world_to_planner(np.array(sc["puck"], dtype=float), P)
    return xR, predict(puck[0:2], puck[3:5], P)


def test_derivatives_against_finite_differences(config, P, nlp):
    rng = np.random.default_rng(1)
    xR, puck = _planner_setup(config, P, "incoming")
    for (s, k) in [(0, 0), (1, 2)]:
        ybar = 2 * s * k * P.w
        w0 = nlp.cold_guess(xR, puck, ybar, 0.6)
        p = nlp.make_params(xR, np.zeros(2), ybar, puck.q, puck.v)
        w0 = w0 + 0.01 * rng.normal(size=w0.size)
        grad = np.array(nlp.grad_f_fun(w0, p)).ravel()
        jac = np.array(nlp.jac_g_fun(w0, p))
        h = 1e-6
        for _ in range(5):
            d = rng.normal(size=w0.size)
            fd_f = (float(nlp.f_fun(w0 + h * d, p)) - float(nlp.f_fun(w0 - h * d, p))) / (2 * h)
            fd_g = (np.array(nlp.g_fun(w0 + h * d, p)) - np.array(nlp.g_fun(w0 - h * d, p))).ravel() / (2 * h)
            assert np.isclose(fd_f, grad @ d, rtol=1e-5, atol=1e-6)
            assert np.allclose(fd_g, jac @ d, rtol=1e-5, atol=1e-6)


@pytest.mark.parametrize("scenario", ["stationary", "incoming", "around"])
def test_cold_start_converges(config, P, nlp, scenario):
    xR, puck = _planner_setup(config, P, scenario)
    sel = ShotSelector(P, nlp)
    plan = sel.update(0.0, xR, np.zeros(2), puck, outer=True)
    assert plan is not None and plan.sol.valid, sel.logs[-1].results

    sol = plan.sol
    assert sol.tf <= min(P.t_max, puck.tau1) + 1e-6  # strike before the puck's next wall hit
    p_c = puck.state_at(sol.tf)[0]
    assert np.isclose(np.linalg.norm(sol.X[0:2, -1] - p_c), P.r_R + P.r_p, atol=1e-4)
    assert abs(sol.terms["e_aim"]) <= P.h_eff + 1e-4
    assert sol.terms["v_n"] >= P.v_min - 1e-4
    # No contact with the puck before t_f
    for k in range(1, P.N):
        pp = puck.state_at(k * sol.tf / P.N)[0]
        assert np.linalg.norm(sol.X[0:2, k] - pp) >= P.r_R + P.r_p - 1e-3


@pytest.mark.slow
def test_closed_loop_scores(config, physics, P, nlp):
    from controllers.shot_mpc.shot_controller import ShotController
    from simulation.episode import run_episode
    from simulation.setup import initial_state

    ctrl = ShotController(config, physics, P, nlp, verbose=False)
    res = run_episode(config, physics, ctrl, initial_state(config, "stationary"), 3.0, stop_on_goal=True,
                      record=False)
    assert res["outcome"] == "goal", res["outcome"]
