"""Test 2: wall-hit events of the puck predictor against a simulator rollout."""

import numpy as np

from conftest import DT, R1_PARK, R2_PARK, rollout
from shot_mpc.params import state_world_to_planner
from shot_mpc.puck_predictor import predict_events


def _events_from_world(puck_world, P, t_max):
    s = state_world_to_planner(np.asarray(puck_world, dtype=float), P)
    return predict_events(s[0:2], s[3:5], P, t_max)


def _flip_steps(v):
    """Indices i where the velocity component changed sign between states i-1 and i."""
    return [i for i in range(1, len(v)) if np.sign(v[i]) != np.sign(v[i - 1])]


def test_side_wall_hits(physics, P):
    puck = [1.0, 0.3, 0.0, -0.5, 1.2, 0.0]
    t_max = 2.0
    states, *_ = rollout(physics, np.array(R1_PARK + R2_PARK + puck), int(t_max / DT))
    ev = _events_from_world(puck, P, t_max)

    flips = _flip_steps(states[:, 16])  # world vy of the puck
    side = [m for m, wall in enumerate(ev.walls) if wall in ("L", "R")]
    assert len(side) == len(flips) >= 2
    for m, i in zip(side, flips):
        assert abs(ev.tau[m] - i * DT) <= DT + 1e-9
        q_sim = state_world_to_planner(states[i, 12:18], P)[0:2]
        assert np.linalg.norm(ev.q[m] - q_sim) <= np.linalg.norm(ev.v[m]) * DT + 2e-3
    # First hit: world top wall, i.e. planner wall R
    assert ev.walls[side[0]] == "R"


def test_end_wall_outside_mouth(physics, P):
    puck = [0.6, 0.2, 0.0, -1.0, 0.0, 0.0]
    states, scored, *_ = rollout(physics, np.array(R1_PARK + R2_PARK + puck), 100)
    ev = _events_from_world(puck, P, 1.0)

    assert ev.walls[1] == "far" and not scored.any()
    i = _flip_steps(states[:, 15])[0]  # world vx of the puck
    assert abs(ev.tau[1] - i * DT) <= DT + 1e-9


def test_goal_crossing(physics, P):
    puck = [0.6, 0.53, 0.0, -1.0, 0.05, 0.0]
    _, scored, *_ = rollout(physics, np.array(R1_PARK + R2_PARK + puck), 100, stop_on_goal=True)
    ev = _events_from_world(puck, P, 1.0)

    assert ev.end == "goal_for" and scored[-1] == 1
    assert abs(ev.t_end - len(scored) * DT) <= DT + 1e-9
