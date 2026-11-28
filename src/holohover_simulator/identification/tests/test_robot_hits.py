import copy

import numpy as np
import pytest

import data
import model
import robot_hits as rh

CFG = data.load_config()
RC = rh.constants(CFG)
P = {"e_n": 0.85, "mu": 0.08, "e_t": 0.35, "m_p": 0.02}


def _random_hit(rng):
    n = rng.normal(size=2)
    n /= np.linalg.norm(n)
    speed = rng.uniform(0.5, 2.5)
    ang = np.radians(rng.uniform(-70, 70))  # between -v and n
    c, s = np.cos(ang), np.sin(ang)
    vp = -speed * np.array([c * n[0] - s * n[1], s * n[0] + c * n[1]])
    return vp, rng.uniform(-10, 10), rng.normal(scale=0.01, size=2), rng.uniform(-0.3, 0.3), n


@pytest.mark.parametrize("seed", range(5))
def test_momentum_and_angular_momentum_are_conserved(seed):
    vp, wp, vr, wr, n = _random_hit(np.random.default_rng(seed))
    vp1, wp1, vr1, wr1 = rh.impact(vp, wp, vr, wr, n, P, RC)
    m_p, m_r = P["m_p"], RC["m_r"]
    np.testing.assert_allclose(m_p * vp + m_r * vr, m_p * vp1 + m_r * vr1, atol=1e-12)
    # about the contact point c = r_robot + r_r n (positions relative to it)
    I_p = RC["kappa"] * m_p * RC["r_p"] ** 2
    cross = lambda r, v: r[0] * v[1] - r[1] * v[0]
    L = lambda vp_, wp_, vr_, wr_: (I_p * wp_ + RC["I_r"] * wr_ + m_p * cross(RC["r_p"] * n, vp_)
                                    + m_r * cross(-RC["r_r"] * n, vr_))
    assert L(vp, wp, vr, wr) == pytest.approx(L(vp1, wp1, vr1, wr1), abs=1e-12)


def test_normal_restitution_of_the_relative_velocity():
    vp, wp, vr, wr, n = _random_hit(np.random.default_rng(7))
    vp1, _, vr1, _ = rh.impact(vp, wp, vr, wr, n, P, RC)
    assert float((vp1 - vr1) @ n) == pytest.approx(-P["e_n"] * float((vp - vr) @ n))


def test_infinitely_heavy_robot_is_the_wall_law():
    rc = {**RC, "m_r": 1e12, "I_r": 1e12}
    wall = model.make_wall("y_min", [0, 1], 0.0)
    for seed in range(5):
        vp, wp, _, _, _ = _random_hit(np.random.default_rng(seed))
        vp = np.array([vp[0], -abs(vp[1])])
        out = model.impact([0, 0, 0, vp[0], vp[1], wp], wall, P, RC["r_p"], RC["kappa"])
        vp1, wp1, _, _ = rh.impact(vp, wp, [0, 0], 0.0, [0, 1], P, rc)
        np.testing.assert_allclose(vp1, out[3:5], atol=1e-9)
        assert wp1 == pytest.approx(out[5], abs=1e-6)


def test_fit_recovers_the_parameters_from_noisy_hits():
    rng = np.random.default_rng(1)
    hits = []
    for _ in range(40):
        vp, wp, vr, wr, n = _random_hit(rng)
        vp1, wp1, vr1, wr1 = rh.impact(vp, wp, vr, wr, n, P, RC)
        hits.append({"n": n, "v_puck_pre": vp, "w_puck_pre": wp, "v_robot_pre": vr, "w_robot_pre": wr,
                     "v_puck_post": vp1 + rng.normal(scale=0.01, size=2), "w_puck_post": wp1 + rng.normal(scale=0.2),
                     "v_robot_post": vr1 + rng.normal(scale=0.005, size=2), "w_robot_post": wr1 + rng.normal(scale=0.1)})
    cfg = copy.deepcopy(CFG)
    p, params, _ = rh.fit(hits, cfg)
    assert p["e_n"] == pytest.approx(P["e_n"], abs=0.02)
    assert p["m_p"] == pytest.approx(P["m_p"], rel=0.05)
    assert p["mu"] == pytest.approx(P["mu"], abs=0.02)
    assert p["e_t"] == pytest.approx(P["e_t"], abs=0.08)
