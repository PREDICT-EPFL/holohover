"""Test 4: a shot aimed at the virtual goal scores after exactly the bounces of B, in time T_B."""

import numpy as np
import pytest

from conftest import DT, R1_PARK, R2_PARK, rollout
from shot_mpc.params import state_planner_to_world
from shot_mpc.model import hit_time, bounce_path, shot_polyline
from shot_mpc.shot_selector import sequences, label, virtual_goal_y

# wall_bounces indices of dynamics.wall_collision: 1 = top (planner wall R), 4 = bottom (planner wall L)
WALL_OF_INDEX = {1: "R", 4: "L"}


@pytest.mark.parametrize("s,k", sequences(2))
def test_virtual_goal_scores(physics, P, s, k):
    p_c = np.array([0.6, 0.1])
    ybar = virtual_goal_y(s, k, P.w)
    d = np.array([P.x_g, ybar]) - p_c
    v = 2.0 * d / np.linalg.norm(d)

    puck = state_planner_to_world(np.array([p_c[0], p_c[1], 0.0, v[0], v[1], 0.0]), P)
    puck[2] = 0.0
    _, scored, bounces, _ = rollout(physics, np.array(R1_PARK + R2_PARK + list(puck)), 300, stop_on_goal=True)

    seq = "".join(WALL_OF_INDEX[j] for b in bounces for j in np.flatnonzero(b))
    assert scored[-1] == 1
    assert seq == ("" if k == 0 else label(s, k))
    T_B = hit_time(P.x_g, p_c[0], v[0], P.gamma)  # side-wall bounces never change v_x
    assert abs(len(scored) * DT - T_B) <= 2 * DT

    points, hits = bounce_path(p_c, v, P, t_end=3.0)
    assert hits == list(seq) + ["goal_for"]

    # Drawn shot: exactly the k bounce vertices of the predictor, ending at the goal centre
    poly = shot_polyline(p_c, v, P)
    assert len(poly) == k + 2
    assert np.allclose(poly, points)
    assert np.allclose(poly[-1], [P.x_g, 0.0], atol=1e-9)
