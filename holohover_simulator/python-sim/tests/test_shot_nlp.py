"""Test 6: derivatives of P(B), cold-start convergence, and a closed-loop smoke test."""

import numpy as np
import pytest

from shot_mpc.params import state_world_to_planner
from shot_mpc.puck_predictor import predict_events
from shot_mpc.shot_selector import ShotSelector


def _planner_setup(config, P, scenario):
    sc = config["shot_mpc"]["scenarios"][scenario]
    xR = state_world_to_planner(np.array(sc["robot1"], dtype=float), P)
    puck = state_world_to_planner(np.array(sc["puck"], dtype=float), P)
    events = predict_events(puck[0:2], puck[3:5], P, t_max=P.t_max + 0.5)
    return xR, events


def test_derivatives_against_finite_differences(config, P, nlp):
    rng = np.random.default_rng(1)
    xR, events = _planner_setup(config, P, "incoming")
    sel = ShotSelector(P, nlp)
    for (s, k) in [(0, 0), (1, 2)]:
        ybar = 2 * s * k * P.w
        w0 = sel._cold_guess(xR, events, 0, ybar, 0.6)
        p = sel._params(xR, np.zeros(3), ybar, events, 0, 0.6)
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
    xR, events = _planner_setup(config, P, scenario)
    sel = ShotSelector(P, nlp)
    plan = sel.update(0.0, xR, np.zeros(3), events, outer=True)
    assert plan is not None and plan.sol.valid, sel.logs[-1].results

    sol = plan.sol
    p_c = events.state_at(sol.tf, 0)[0]
    assert np.isclose(np.linalg.norm(sol.X[0:2, -1] - p_c), P.r_R + P.r_p, atol=1e-4)
    assert abs(sol.terms["e_aim"]) <= P.h_eff + 1e-4
    assert sol.terms["v_n"] >= P.v_min - 1e-4
    # No contact with the puck before t_f
    for k in range(1, P.N):
        pp = events.state_at(k * sol.tf / P.N, 0)[0]
        assert np.linalg.norm(sol.X[0:2, k] - pp) >= P.r_R + P.r_p - 1e-3


@pytest.mark.slow
def test_closed_loop_scores(config, physics, P, nlp):
    from run_shot import initial_state, run_episode

    res = run_episode(config, physics, P, nlp, initial_state(config, "stationary"), 3.0,
                      stop_on_goal=True, verbose=False, record=False)
    assert res["outcome"] == "goal", res["outcome"]
