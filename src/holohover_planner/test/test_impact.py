"""The robot-puck impact with Coulomb friction (model.impact): frictionless limit, sliding on the friction
cone, sticking (the contact point stops sliding, spins included), momentum, numpy against CasADi."""

import casadi as ca
import numpy as np

from holohover_planner.shot_mpc.model import CLIP_SMOOTH, impact

N = np.array([np.cos(0.3), np.sin(0.3)])  # contact normal, robot -> puck
T = np.array([-N[1], N[0]])


def _shot(P, cut_deg, speed=1.0, v_p=(0.0, 0.0)):
    """Robot velocity at cut_deg from the normal, and the impact with the planner's parameters."""
    c = np.radians(cut_deg)
    v_R = speed * (np.cos(c) * N + np.sin(c) * T)
    v_p = np.array(v_p, dtype=float)
    return v_R, v_p, impact(v_R, v_p, N, P.kappa, P.m_p / P.m_R, P.mu, P.e)


def test_frictionless(P):
    v_R, v_p = np.array([0.9, 0.4]), np.array([0.1, -0.2])
    v_n, v_p_plus, v_R_plus = impact(v_R, v_p, N, P.kappa, P.m_p / P.m_R)
    assert v_n == (v_R - v_p) @ N
    assert np.allclose(v_p_plus, v_p + P.kappa * v_n * N, rtol=0, atol=1e-15)
    assert np.allclose(v_R_plus, v_R - P.kappa * P.m_p / P.m_R * v_n * N, rtol=0, atol=1e-15)


def test_sliding_on_the_cone(P):
    """A cut beyond the stick limit: the puck turns by atan(mu) toward the robot's sliding direction."""
    for cut in (-40.0, 25.0, 60.0):
        v_R, v_p, (v_n, v_p_plus, _) = _shot(P, cut)
        dv = v_p_plus - v_p
        f = dv @ T / P.kappa
        assert abs(f - np.sign(cut) * P.mu * v_n) <= CLIP_SMOOTH
        assert np.isclose(dv @ N, P.kappa * v_n)


def test_sticking(P):
    """A cut inside the stick limit: the tangential impulse stops the sliding at the contact point. A tangential
    impulse also spins each disc (I = m r^2 / 2), so it moves its contact point three times its centre."""
    for cut in (-3.0, 0.0, 4.0):
        v_R, v_p, (v_n, v_p_plus, v_R_plus) = _shot(P, cut)
        u_t = (v_R - v_p) @ T
        assert abs(u_t) < 3 * (1 + P.e) * P.mu * v_n - 5 * CLIP_SMOOTH  # inside the cone
        u_t_plus = u_t + 3 * (v_R_plus - v_R) @ T - 3 * (v_p_plus - v_p) @ T
        assert abs(u_t_plus) < 1e-3 * abs(u_t) + 1e-9


def test_momentum(P):
    for cut in (-30.0, 2.0, 50.0):
        v_R, v_p, (_, v_p_plus, v_R_plus) = _shot(P, cut, v_p=(0.3, -0.5))
        assert np.allclose(P.m_R * (v_R_plus - v_R) + P.m_p * (v_p_plus - v_p), 0.0, atol=1e-12)


def test_casadi_matches_numpy(P):
    vr, vp, n = ca.SX.sym("vr", 2), ca.SX.sym("vp", 2), ca.SX.sym("n", 2)
    fun = ca.Function("impact", [vr, vp, n], list(impact(vr, vp, n, P.kappa, P.m_p / P.m_R, P.mu, P.e)))
    for cut in (-30.0, 2.0, 50.0):
        v_R, v_p, ref = _shot(P, cut, v_p=(0.3, -0.5))
        out = fun(v_R, v_p, N)
        for a, b in zip(out, ref):
            assert np.allclose(np.array(a).ravel(), np.ravel(b), rtol=0, atol=1e-12)
