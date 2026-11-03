"""Derivatives of P(B) and cold-start convergence (from python-sim tests/test_shot_nlp.py), with the
scenarios of python-sim's config.json given in its world frame (table frame + (X/2, W/2))."""

import numpy as np
import pytest

from holohover_planner.shot_mpc.model import predict
from holohover_planner.shot_mpc.shot_selector import ShotSelector, virtual_goal

# (robot1, puck): x, y, vx, vy in python-sim's world frame
SCENARIOS = {
    "stationary": ([1.9, 0.85, 0.0, 0.0], [1.45, 0.40, 0.0, 0.0]),
    "incoming": ([1.81, 0.53, 0.0, 0.0], [0.6, 0.75, 1.0, -0.25]),
    "around": ([1.3, 0.50, 0.0, 0.0], [1.6, 0.55, 0.0, 0.0]),
}


def _planner_setup(P, frame, scenario):
    """Robot state (x, y, vx, vy) and puck flight of a scenario, planner frame."""
    def to_planner(s):
        s = np.asarray(s, dtype=float)
        return np.concatenate([frame.pos_to_planner(s[0:2] - [0.5 * P.X, 0.5 * P.W]), frame.vec_to_planner(s[2:4])])

    robot, puck = (to_planner(s) for s in SCENARIOS[scenario])
    return robot, predict(puck[0:2], puck[2:4], P)


def test_derivatives_against_finite_differences(P, frame, nlp):
    rng = np.random.default_rng(1)
    xR, puck = _planner_setup(P, frame, "incoming")
    for (s, k) in [(0, 0), (1, 2)]:
        ybar, hbar = virtual_goal(s, k, P)
        w0 = nlp.cold_guess(xR, puck, ybar, 0.6)
        p = nlp.make_params(xR, np.zeros(2), ybar, hbar, puck.q, puck.v)
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


@pytest.mark.parametrize("bad", [np.nan, np.inf, 1e6])
def test_unsolvable_input_is_invalid_not_a_hang(P, frame, nlp, bad):
    xR, puck = _planner_setup(P, frame, "incoming")
    ybar, hbar = virtual_goal(0, 0, P)
    p = nlp.make_params(xR, np.zeros(2), ybar, hbar, puck.q, puck.v)
    w0 = nlp.init_slacks(nlp.cold_guess(xR, puck, ybar, 0.6), p)
    w0[nlp.idx["T"]] = 0.6
    w0[nlp.idx["phi"]] = bad
    assert not nlp.solve(w0, p, P.t_min, 1.0).valid
    p_bad = p.copy()
    p_bad[0] = bad
    assert not nlp.solve(nlp.init_slacks(nlp.cold_guess(xR, puck, ybar, 0.6), p), p_bad, P.t_min, 1.0).valid
    assert nlp.solve(nlp.init_slacks(nlp.cold_guess(xR, puck, ybar, 0.6), p), p, P.t_min, 1.0).valid  # still works after


@pytest.mark.parametrize("scenario", list(SCENARIOS))
def test_cold_start_converges(P, frame, nlp, scenario):
    xR, puck = _planner_setup(P, frame, scenario)
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


def test_pool_matches_serial(P, frame, nlp):
    """Outer update in spawned worker processes: bit-identical to the serial solves."""
    from holohover_planner.shot_mpc.shot_nlp import ShotNLP

    xR, puck = _planner_setup(P, frame, "incoming")
    serial = ShotSelector(P, nlp)
    pooled_nlp = ShotNLP(P, verbose=False)
    pooled_nlp.start_pool(2)
    try:
        pooled = ShotSelector(P, pooled_nlp)
        for t in (0.0, 0.05):
            a = serial.update(t, xR, np.zeros(2), puck, outer=True)
            b = pooled.update(t, xR, np.zeros(2), puck, outer=True)
            assert pooled.logs[-1].parallel
            assert a.key == b.key and np.array_equal(a.sol.w, b.sol.w)
    finally:
        pooled_nlp.pool.close()
