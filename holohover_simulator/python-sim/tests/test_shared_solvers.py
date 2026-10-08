"""Objects built once and shared by every episode: the worker pool of the outer updates
(shot_mpc/solve_pool.py), the tracker and the defence PD controllers."""

import os
import subprocess
import sys
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parents[1]

# A cold then a warm outer update of each scenario, solved in 2 worker processes and serially
POOL_CHECK = """
import sys
sys.path.insert(0, ".")
import numpy as np
from controllers.shot_mpc.params import load_config, ShotParams, XY, state_world_to_planner
from controllers.shot_mpc.model import predict
from controllers.shot_mpc.shot_nlp import ShotNLP
from controllers.shot_mpc.shot_selector import ShotSelector

config = load_config("config/config_shot.json")
P = ShotParams.from_config(config)
nlp = ShotNLP(P, jit=False, verbose=False)
pool = nlp.start_pool(2)
for scenario in ("stationary", "incoming", "around"):
    sc = config["shot_mpc"]["scenarios"][scenario]
    xR = state_world_to_planner(np.array(sc["robot1"], dtype=float), P)[XY]
    puck = state_world_to_planner(np.array(sc["puck"], dtype=float), P)
    puck = predict(puck[0:2], puck[3:5], P)
    runs = []
    for p in (pool, None):
        nlp.pool = p
        sel = ShotSelector(P, nlp)
        chosen = [sel.update(t, xR, np.zeros(2), puck, outer=True) for t in (0.0, 0.05)]
        runs.append(([c.key for c in chosen], {k: v.sol for k, v in sel.candidates.items()}))
    (keys_par, par), (keys_ser, ser) = runs
    assert keys_par == keys_ser and par.keys() == ser.keys(), scenario
    for k in ser:
        assert np.array_equal(par[k].w, ser[k].w) and par[k].cost == ser[k].cost, (scenario, k)
pool.close()
print("identical")
"""


def test_pool_matches_serial():
    # A fresh interpreter: the pool must fork before JAX starts its threads, which the other
    # tests of the session may have done already
    res = subprocess.run([sys.executable, "-c", POOL_CHECK], cwd=SIM_DIR, capture_output=True, text=True,
                         timeout=600, env={**os.environ, "JAX_PLATFORMS": "cpu"})
    assert res.returncode == 0 and res.stdout.strip().endswith("identical"), res.stderr[-2000:]


def test_episodes_share_tracker_and_pd(config, physics, P, nlp):
    from run_shot import Episode, initial_state
    from controllers.shot_mpc.shot_controller import make_tracker

    tracker = make_tracker(config, P)
    eps = [Episode(config, physics, P, nlp, initial_state(config, "stationary"), verbose=False, tracker=tracker)
           for _ in range(2)]
    assert eps[0].ctrl.tracker is eps[1].ctrl.tracker is tracker
    assert eps[0].ctrl.def_controller is eps[1].ctrl.def_controller  # compiled once per process
    for _ in range(10):
        eps[0].step(record=False)
    assert len(eps[0].ctrl.tracker_times) > 0 and eps[1].ctrl.tracker_times == []  # timings stay per episode
