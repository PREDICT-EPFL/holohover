"""The virtual goal of a bounce sequence (shot_selector.virtual_goal) against the planner's own bounce model
(model.bounce_path, flight_after), for elastic walls and walls with restitution: a puck aimed at the goal
image scores after exactly the bounces of B, on the goal centre and in time T_B; one aimed at an edge of the
image's mouth crosses the goal line at an edge of the mouth."""

from dataclasses import replace

import numpy as np
import pytest

from holohover_planner.shot_mpc.model import bounce_path, flight_after, hit_time
from holohover_planner.shot_mpc.shot_selector import label, sequences, virtual_goal

IMPACTS = ([0.6, 0.1], [1.2, -0.3], [0.3, 0.4])  # puck centre at the impact, planner frame


@pytest.mark.parametrize("e_w", [1.0, 0.9, 0.7])
@pytest.mark.parametrize("s,k", sequences(2))
def test_virtual_goal_scores(P, e_w, s, k):
    P = replace(P, e_w=e_w)
    ybar, hbar = virtual_goal(s, k, P)
    bounces = [] if k == 0 else list(label(s, k))
    for p_c in map(np.array, IMPACTS):
        for edge in (0.0, 1.0, -1.0):  # aim at the image's centre, then at its mouth edges
            d = np.array([P.x_g, ybar + edge * hbar]) - p_c
            v = 2.0 * d / np.linalg.norm(d)
            points, hits = bounce_path(p_c, v, P, t_end=10.0)
            assert hits == bounces + ["goal_for"]
            assert abs(points[-1][0] - P.x_g) < 1e-9
            assert abs(abs(points[-1][1]) - abs(edge) * P.h_eff) < 1e-9
            if edge == 0.0:
                # Side-wall bounces never change v_x: the goal line is reached at T_B of the unfolded shot
                T_B = hit_time(P.x_g, p_c[0], v[0], P.gamma)
                q, _ = flight_after(p_c, v, P, T_B - 1e-9)
                assert np.allclose(q, [P.x_g, 0.0], atol=1e-6)


def test_wall_bounce_velocity(P):
    """A side-wall bounce keeps v_x and reverses v_y scaled by e_w; an end wall does the same to v_x."""
    P = replace(P, e_w=0.9)
    t = 0.8
    decay = np.exp(-P.gamma * t)
    _, v = flight_after(np.array([1.0, 0.0]), np.array([0.3, 1.0]), P, t)  # wall L at 0.54 s
    assert np.allclose(v, [0.3 * decay, -0.9 * decay])
    _, v = flight_after(np.array([1.5, 0.3]), np.array([1.0, 0.0]), P, t)  # far wall, outside the mouth, at 0.66 s
    assert np.allclose(v, [-0.9 * decay, 0.0])
