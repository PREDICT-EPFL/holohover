"""Closed-loop smoke test of the shot MPC in python-sim (the NLP itself is tested in holohover_planner)."""

import pytest


@pytest.mark.slow
def test_closed_loop_scores(config, physics, P, nlp):
    from controllers.shot_mpc.controller import ShotController, make_tracker
    from simulation.episode import run_episode
    from simulation.setup import initial_state

    ctrl = ShotController(config, P, nlp, make_tracker(config), verbose=False)
    res = run_episode(config, physics, ctrl, initial_state(config, "stationary"), 3.0, stop_on_goal=True,
                      record=False)
    assert res["outcome"] == "goal", res["outcome"]
