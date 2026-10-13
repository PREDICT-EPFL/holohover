"""Test 2: first wall hit of the puck predictor, and the drawn bounce path, against a simulator rollout."""

import numpy as np

from conftest import DT, R1_PARK, R2_PARK, rollout
from controllers.shot_mpc.params import state_world_to_planner
from controllers.shot_mpc.model import predict, bounce_path, flight_after


def _planner(puck_world, P):
    """Planner-frame puck position and velocity."""
    s = state_world_to_planner(np.asarray(puck_world, dtype=float), P)
    return s[0:2], s[3:5]


def _flip_steps(v):
    """Indices i where the velocity component changed sign between states i-1 and i."""
    return [i for i in range(1, len(v)) if np.sign(v[i]) != np.sign(v[i - 1])]


def test_first_side_wall_hit(physics, P):
    puck = [1.0, 0.3, 0.0, -0.5, 1.2, 0.0]
    states, *_ = rollout(physics, np.array(R1_PARK + R2_PARK + puck), 100)
    flight = predict(*_planner(puck, P), P)

    i = _flip_steps(states[:, 16])[0]  # world vy of the puck
    assert flight.wall == "R"  # world top wall
    assert abs(flight.tau1 - i * DT) <= DT + 1e-9
    q_hit, v_hit = flight.state_at(flight.tau1)
    assert abs(q_hit[1] + P.w) < 1e-9
    q_sim = state_world_to_planner(states[i, 12:18], P)[0:2]
    assert np.linalg.norm(q_hit - q_sim) <= np.linalg.norm(v_hit) * DT + 2e-3


def test_end_wall_outside_mouth(physics, P):
    puck = [0.6, 0.2, 0.0, -1.0, 0.0, 0.0]
    states, scored, *_ = rollout(physics, np.array(R1_PARK + R2_PARK + puck), 100)
    flight = predict(*_planner(puck, P), P)

    assert flight.wall == "far" and not scored.any()
    i = _flip_steps(states[:, 15])[0]  # world vx of the puck
    assert abs(flight.tau1 - i * DT) <= DT + 1e-9


def test_goal_crossing(physics, P):
    puck = [0.6, 0.53, 0.0, -1.0, 0.05, 0.0]
    _, scored, *_ = rollout(physics, np.array(R1_PARK + R2_PARK + puck), 100, stop_on_goal=True)
    q, v = _planner(puck, P)
    flight = predict(q, v, P)

    assert flight.wall == "far" and scored[-1] == 1
    assert abs(flight.tau1 - len(scored) * DT) <= DT + 1e-9
    points, hits = bounce_path(q, v, P, t_end=1.0)
    assert hits == ["goal_for"]
    assert np.allclose(points[-1], flight.state_at(flight.tau1)[0])


def test_bounce_path_side_walls(physics, P):
    """The drawn path has every side-wall hit of a 2 s rollout, alternating and in place."""
    puck = [1.0, 0.3, 0.0, -0.5, 1.2, 0.0]
    t_end = 2.0
    states, *_ = rollout(physics, np.array(R1_PARK + R2_PARK + puck), int(t_end / DT))
    q, v = _planner(puck, P)
    points, hits = bounce_path(q, v, P, t_end)

    flips = _flip_steps(states[:, 16])
    assert len(hits) == len(flips) >= 2
    assert hits == ["R", "L"] * (len(hits) // 2) + ["R"] * (len(hits) % 2)
    for q_hit, i in zip(points[1:-1], flips):  # the last point is the position at t_end
        q_sim = state_world_to_planner(states[i, 12:18], P)[0:2]
        assert np.linalg.norm(q_hit - q_sim) <= np.linalg.norm(v) * DT + 2e-3


def test_flight_after_side_wall(physics, P):
    """Position and velocity after a side-wall hit, against the rollout."""
    puck = [1.0, 0.3, 0.0, -0.5, 1.2, 0.0]
    steps = 80
    states, *_ = rollout(physics, np.array(R1_PARK + R2_PARK + puck), steps)
    q, v = flight_after(*_planner(puck, P), P, steps * DT)

    assert len(_flip_steps(states[:, 16])) == 1
    s = state_world_to_planner(states[-1, 12:18], P)
    assert np.linalg.norm(q - s[0:2]) < 3e-3  # the rollout's Euler friction drifts by ~v gamma DT t / 2
    assert np.linalg.norm(v - s[3:5]) < 1e-3
