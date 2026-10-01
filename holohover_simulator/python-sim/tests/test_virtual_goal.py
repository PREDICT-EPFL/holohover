"""Test 4: a shot aimed at the virtual goal scores after exactly the bounces of B, in time T_B."""

import numpy as np
import pytest

from conftest import DT, R1_PARK, R2_PARK, rollout
from shot_mpc.params import state_planner_to_world
from shot_mpc.bounce_geometry import sequences, label, virtual_goal_y, time_to_goal, shot_polyline
from shot_mpc.puck_predictor import predict_events

# wall_bounces indices of dynamics.wall_collision: 1 = top (planner wall R), 4 = bottom (planner wall L)
WALL_OF_INDEX = {1: "R", 4: "L"}


@pytest.mark.parametrize("s,k", sequences(2))
def test_virtual_goal_scores(physics, P, s, k):
    p_c = np.array([0.6, 0.1])
    ybar = virtual_goal_y(s, k, P.w, P.y_g)
    d = np.array([P.x_g, ybar]) - p_c
    v = 2.0 * d / np.linalg.norm(d)

    puck = state_planner_to_world(np.array([p_c[0], p_c[1], 0.0, v[0], v[1], 0.0]), P)
    puck[2] = 0.0
    _, scored, bounces, _ = rollout(physics, np.array(R1_PARK + R2_PARK + list(puck)), 300, stop_on_goal=True)

    seq = "".join(WALL_OF_INDEX[j] for b in bounces for j in np.flatnonzero(b))
    assert scored[-1] == 1
    assert seq == ("" if k == 0 else label(s, k))
    T_B = time_to_goal(p_c[0], v[0], P.x_g, P.gamma)
    assert abs(len(scored) * DT - T_B) <= 2 * DT

    ev = predict_events(p_c, v, P, t_max=3.0)
    assert ev.end == "goal_for" and "".join(ev.walls[1:]) == seq
    assert abs(ev.t_end - T_B) < 1e-9

    # Drawn shot: exactly the k bounce vertices of the predictor, ending at the goal centre
    poly = shot_polyline(p_c, v, P)
    assert len(poly) == k + 2
    assert np.allclose(poly[1:-1], ev.q[1:])
    assert np.allclose(poly[-1], [P.x_g, P.y_g], atol=1e-9)
