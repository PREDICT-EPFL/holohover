"""ShotPlanner: a contact drops the plan and pauses the planning for the cooldown; a reset starts afresh."""

import numpy as np
import pytest

from holohover_planner.shot_mpc.model import discrete_matrices
from holohover_planner.shot_planner import Body, ShotPlanner, advance


def bodies(t):
    """h0 at the experiment's start, the puck at rest in front of it (table frame)."""
    return Body(t, 0.835, 0.32, 0.0, 0.0), Body(t, 0.385, -0.13, 0.0, 0.0)


def test_contact_cooldown_and_reset(P, frame, nlp):
    planner = ShotPlanner(P, frame, nlp)
    plans = planner.tick(0.0, *bodies(0.0))
    assert plans is not None and plans.chosen >= 0 and planner.plan is not None

    none = planner.on_contact(0.1)
    assert none.chosen == -1 and all(c is None for c in none.candidates)
    assert planner.on_contact(0.2) is None  # during the cooldown: ignored
    assert planner.tick(0.15, *bodies(0.15)) is None and planner.plan is None  # dropped, no planning
    assert planner.attack_after == 0.1 + P.contact_cooldown

    planner.reset()
    plans = planner.tick(0.2, *bodies(0.2))  # no cooldown any more, and an outer update
    assert plans is not None and plans.chosen >= 0
    assert planner.selector.logs[-1].kind == "outer"


def test_advance(P, frame, nlp):
    """The robot is advanced to the tick's time under the followed plan's inputs."""
    planner = ShotPlanner(P, frame, nlp)
    plans = planner.tick(0.0, *bodies(0.0))
    assert plans.plan.t0 == pytest.approx(0.0)

    # advance() is the exact rollout of the plan's ZOH inputs, from a node or from inside an interval
    p = planner.plan
    A_d, B_d = discrete_matrices(p.dt, P.gamma)
    nodes = [p.sol.X[:, 0]]
    for k in range(P.N):
        nodes.append(A_d @ nodes[-1] + B_d @ p.sol.U[:, k])
    for k in (1, 7, P.N):
        assert np.allclose(advance(p.sol.X[:, 0], p.t0, p.t0 + k * p.dt, p, None, P.gamma), nodes[k], atol=1e-12)
    x_mid = advance(p.sol.X[:, 0], p.t0, p.t0 + 2.4 * p.dt, p, None, P.gamma)
    assert np.allclose(advance(x_mid, p.t0 + 2.4 * p.dt, p.t0 + 9 * p.dt, p, None, P.gamma), nodes[9], atol=1e-12)
