from types import SimpleNamespace

import numpy as np

from holohover_planner.shot_planner import candidates
from holohover_planner.shot_mpc.shot_selector import ShotSelector, label


def _plan(key, valid, t0=0.0):
    return SimpleNamespace(key=key, label=label(*key), t0=t0, sol=SimpleNamespace(valid=valid, cost=1.0))


def test_one_entry_per_sequence_in_order(P, nlp):
    sel = ShotSelector(P, nlp)
    out = candidates(sel, None)
    assert [name for name, _ in out] == ["direct", "L", "R", "LR", "RL"]
    assert all(entry is None for _, entry in out)


def test_followed_plan_valid_and_invalid_candidates(P, nlp):
    sel = ShotSelector(P, nlp)
    followed = _plan((1, 1), True, t0=0.0)
    sel.candidates[(1, 1)] = _plan((1, 1), False, t0=0.05)  # its latest solve failed: the followed plan stays
    sel.candidates[(0, 0)] = _plan((0, 0), True)
    sel.candidates[(-1, 1)] = _plan((-1, 1), False)
    out = dict(candidates(sel, followed))
    assert out["L"] is followed
    assert out["direct"] is sel.candidates[(0, 0)]
    assert out["R"] is None and out["LR"] is None and out["RL"] is None


def test_real_update_gives_valid_candidates(P, frame, nlp):
    from holohover_planner.shot_mpc.model import predict

    sel = ShotSelector(P, nlp)
    robot = np.concatenate([frame.pos_to_planner([0.835, 0.32]), [0.0, 0.0]])
    puck = frame.pos_to_planner([0.385, -0.13])
    plan = sel.update(0.0, robot, np.zeros(2), predict(puck, np.zeros(2), P), outer=True)
    out = candidates(sel, plan)
    valid = [entry for _, entry in out if entry is not None]
    assert plan in valid and all(entry.sol.valid for entry in valid)
    assert plan.sol.cost == min(entry.sol.cost for entry in valid)
