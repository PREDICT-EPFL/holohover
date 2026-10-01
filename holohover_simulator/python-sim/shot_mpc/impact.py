"""
Frictionless impulsive robot-puck impact along the contact normal n (Section 4).

    v_n  = (v_R - v_p) . n
    v_p+ = v_p + kappa v_n n
    v_R+ = v_R - kappa (m_p / m_R) v_n n,   kappa = (1 + e) m_R / (m_R + m_p)

The functions use only elementwise operations so they work on numpy arrays and
on CasADi symbols alike.
"""

import numpy as np
import casadi as ca


def kappa(m_R, m_p, e=1.0):
    return (1.0 + e) * m_R / (m_R + m_p)


def _vec2(x, y):
    if isinstance(x, (ca.SX, ca.MX)) or isinstance(y, (ca.SX, ca.MX)):
        return ca.vertcat(x, y)
    return np.array([x, y], dtype=float).ravel()


def impact(v_R, v_p, n, kap, mass_ratio):
    """Return (v_n, v_p_plus, v_R_plus); mass_ratio = m_p / m_R."""
    v_n = (v_R[0] - v_p[0]) * n[0] + (v_R[1] - v_p[1]) * n[1]
    v_p_plus = _vec2(v_p[0] + kap * v_n * n[0], v_p[1] + kap * v_n * n[1])
    v_R_plus = _vec2(v_R[0] - kap * mass_ratio * v_n * n[0], v_R[1] - kap * mass_ratio * v_n * n[1])
    return v_n, v_p_plus, v_R_plus


def invert(sigma, d, v_p_minus, kap):
    """
    Contact normal and closing speed that send the puck along unit d at speed sigma.

    Requiring v_p+ = sigma d gives kappa v_n n = sigma d - v_p-.
    Returns (n, v_n, phi) with v_n > 0 by construction.
    """
    diff = sigma * np.asarray(d, dtype=float) - np.asarray(v_p_minus, dtype=float)
    norm = np.linalg.norm(diff)
    n = diff / norm
    return n, norm / kap, float(np.arctan2(n[1], n[0]))
