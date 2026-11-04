"""Objects built once and shared by every episode: the tracker (the worker pool of the outer updates is
tested in holohover_planner)."""


def test_episodes_share_tracker(config, physics, P, nlp):
    from controllers.shot_mpc.controller import ShotController, make_tracker
    from simulation.episode import Episode
    from simulation.setup import initial_state

    tracker = make_tracker(config)
    eps = [Episode(config, physics, ShotController(config, P, nlp, tracker, verbose=False),
                   initial_state(config, "stationary"))
           for _ in range(2)]
    ctrls = [ep.ctrl.controller for ep in eps]
    assert ctrls[0].tracker is ctrls[1].tracker is tracker
    for _ in range(10):
        eps[0].step(record=False)
    assert len(ctrls[0].tracker_times) > 0 and ctrls[1].tracker_times == []  # timings stay per episode
