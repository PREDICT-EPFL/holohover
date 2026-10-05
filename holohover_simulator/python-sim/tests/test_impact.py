"""Test 3: impact map, its inversion, and one simulator collision."""

from dataclasses import replace

import numpy as np
import jax.numpy as jnp

from conftest import DT, R2_PARK
from dynamics import physics_step
from shot_mpc.model import impact, invert


def test_kappa_sanity(P):
    assert abs(replace(P, m_R=0.140, m_p=0.010, e=1.0).kappa - 1.8667) < 1e-4


def test_inversion_round_trip(P):
    kap, ratio = P.kappa, P.m_p / P.m_R
    v_minus = np.array([-0.4, 0.3])
    d = np.array([0.8, -0.6])
    n, v_n, phi = invert(1.5, d, v_minus, kap)
    assert v_n > 0 and np.isclose(np.arctan2(n[1], n[0]), phi)
    v_R = v_minus + v_n * n + 0.7 * np.array([-n[1], n[0]])  # tangential part does not matter
    v_n2, v_p_plus, _ = impact(v_R, v_minus, n, kap, ratio)
    assert np.isclose(v_n2, v_n)
    assert np.allclose(v_p_plus, 1.5 * d)


def test_against_simulator_collision(physics, P):
    a = np.deg2rad(30.0)
    r = np.array([1.5, 0.5])
    p = r + 0.106 * np.array([np.cos(a), np.sin(a)])  # just apart; they overlap after one step
    v_R, v_p = np.array([0.8, 0.3]), np.array([-0.2, -0.1])
    state = np.concatenate([[*r, 0, *v_R, 0], R2_PARK, [*p, 0, *v_p, 0]])
    s, c1, *_ = physics_step(jnp.asarray(state, dtype=jnp.float32), jnp.zeros(6), physics, DT)
    s = np.asarray(s, dtype=float)
    assert bool(c1)

    # Replicate the simulator: integrate, damp, then collide along the centre line
    damp = 1 - P.gamma * DT
    r1, p1 = r + v_R * DT, p + v_p * DT
    n = (p1 - r1) / np.linalg.norm(p1 - r1)
    _, v_p_plus, v_R_plus = impact(v_R * damp, v_p * damp, n, P.kappa, P.m_p / P.m_R)
    assert np.allclose(s[15:17], v_p_plus, atol=1e-5)
    assert np.allclose(s[3:5], v_R_plus, atol=1e-5)
