from pathlib import Path

import pytest

from holohover_planner.shot_mpc.params import Frame, ShotParams, load_parameters

# The node's parameters: the values of the tests too (the code has no defaults)
CONFIG = Path(__file__).resolve().parents[2] / "holohover_utils" / "config" / "common" / "shot_planner_config.yaml"


@pytest.fixture(scope="session")
def P():
    return ShotParams.from_parameters(load_parameters(CONFIG))


@pytest.fixture(scope="session")
def frame(P):
    return Frame(P, "left")


@pytest.fixture(scope="session")
def nlp(P):
    from holohover_planner.shot_mpc.shot_nlp import ShotNLP

    return ShotNLP(P, verbose=False)
