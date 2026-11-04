import numpy as np
import pytest
from scipy.integrate import solve_ivp

import model

P = {"c_v": 0.3, "g_x": 0.05, "g_y": -0.08, "c_w": 0.5, "c_wc": 1.0, "e_n": 0.8, "mu": 0.2, "e_t": 0.4}
R, K = 0.035, 0.5
BOX = [model.make_wall("x_min", [1, 0], 0.0), model.make_wall("x_max", [-1, 0], -2.0),
       model.make_wall("y_min", [0, 1], 0.0), model.make_wall("y_max", [0, -1], -1.0)]


def _ode(p):
    def f(t, s):
        w = s[5]
        return [s[3], s[4], w, p["g_x"] - p["c_v"] * s[3], p["g_y"] - p["c_v"] * s[4],
                -p["c_w"] * w - p["c_wc"] * np.sign(w)]
    return f


@pytest.mark.parametrize("p", [P, {**P, "c_v": 0.0}, {**P, "c_wc": 0.0}])
def test_flight_matches_numerical_integration(p):
    s0 = [0.1, 0.2, 0.3, 0.8, -0.5, 3.0]
    t = np.linspace(0, 1.5, 50)
    num = solve_ivp(_ode(p), (0, 1.5), s0, t_eval=t, rtol=1e-11, atol=1e-12).y.T
    np.testing.assert_allclose(model.flight(s0, t, p), num, atol=1e-8)


def test_spin_stops_with_coulomb_drag():
    st = model.flight([0, 0, 0, 0, 0, 2.0], [100.0], P)[0]
    assert st[5] == 0.0


def _slip(state, wall):
    n = wall["normal"]
    t = np.array([-n[1], n[0]])
    return state[3:5] @ n, state[3:5] @ t - R * state[5]


def test_impact_stick_reverses_slip_by_e_t():
    wall = BOX[2]
    before = np.array([0, 0, 0, 0.5, -1.0, 2.0])
    after = model.impact(before, wall, {**P, "mu": 10.0}, R, K)
    vn0, ut0 = _slip(before, wall)
    vn1, ut1 = _slip(after, wall)
    assert vn1 == pytest.approx(-P["e_n"] * vn0)
    assert ut1 == pytest.approx(-P["e_t"] * ut0)


def test_impact_slide_is_coulomb_limited():
    wall = BOX[2]
    before = np.array([0, 0, 0, 2.0, -0.5, 0.0])
    after = model.impact(before, wall, P, R, K)
    vn0, ut0 = _slip(before, wall)
    _, ut1 = _slip(after, wall)
    limit = P["mu"] * (1 + P["e_n"]) * -vn0 * (1 + K) / K
    assert abs(ut0) > limit
    assert ut1 == pytest.approx(ut0 - np.sign(ut0) * (1 + P["e_t"]) * limit)
    # and matches the curve used in the plots
    assert ut1 / ut0 == pytest.approx(model.tangential_ratio(abs(ut0) / -vn0, P["e_n"], P["mu"], P["e_t"], K))


def test_impact_never_creates_energy():
    rng = np.random.default_rng(0)
    for _ in range(5000):
        p = {**P, "mu": rng.uniform(0, 2), "e_n": rng.uniform(0, 1), "e_t": rng.uniform(0, 1)}
        s = np.r_[0, 0, 0, rng.normal(0, 1), -abs(rng.normal(0, 1)), rng.normal(0, 30)]
        o = model.impact(s, BOX[2], p, R, K)
        energy = lambda x: x[3] ** 2 + x[4] ** 2 + K * R * R * x[5] ** 2  # noqa: E731
        assert energy(o) <= energy(s) + 1e-12


@pytest.mark.parametrize("p", [P, {**P, "c_v": 0.0, "g_y": -0.5}, {**P, "e_n": 0.2, "g_y": -0.6},
                               {**P, "c_v": 0.0, "g_x": 0.3, "g_y": -0.4, "e_n": 0.95}])
def test_rollout_stays_inside_the_walls(p):
    t = np.linspace(0, 20, 8000)
    st = model.rollout([1, 0.5, 0, 1.5, 0.9, 1], t, p, BOX, R, K)
    assert st[:, 0].min() > -1e-9 and st[:, 0].max() < 2 + 1e-9
    assert st[:, 1].min() > -1e-9 and st[:, 1].max() < 1 + 1e-9


def test_rollout_turns_back_before_the_wall():
    # the tilt stops the puck 4.5 cm before y_max
    st, imp = model.rollout([1, 0.9, 0, 0, 0.3, 0], np.linspace(0, 0.5, 100), {**P, "c_v": 0.0, "g_y": -1.0},
                            BOX, R, K, return_impacts=True)
    assert not imp
    assert st[:, 1].max() == pytest.approx(0.945, abs=1e-3)


def test_rollout_impact_time_is_exact():
    p = {**P, "g_x": 0.0, "g_y": 0.0, "c_v": 0.0}
    _, imp = model.rollout([1, 0.5, 0, 0, -1.0, 0], [2.0], p, BOX, R, K, return_impacts=True)
    assert imp[0][0] == pytest.approx(0.5, abs=1e-12)
