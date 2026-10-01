import os
import sys
from pathlib import Path

os.environ.setdefault("JAX_PLATFORMS", "cpu")
SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))

import numpy as np
import jax.numpy as jnp
import pytest

from dynamics import physics_step
from shot_mpc.params import load_config, physics_from_config, ShotParams

DT = 0.01
# Robots parked in the top corners (world frame), out of the tested puck paths
R1_PARK = [2.05, 0.97, 0.0, 0.0, 0.0, 0.0]
R2_PARK = [0.08, 0.97, 0.0, 0.0, 0.0, 0.0]
PUCK_PARK = [0.3, 0.9, 0.0, 0.0, 0.0, 0.0]


def pytest_configure(config):
    config.addinivalue_line("markers", "slow: closed-loop simulations (deselect with -m 'not slow')")


@pytest.fixture(scope="session")
def config():
    return load_config(SIM_DIR / "config" / "config_shot.json")


@pytest.fixture(scope="session")
def physics(config):
    return physics_from_config(config)


@pytest.fixture(scope="session")
def P(config):
    return ShotParams.from_config(config)


@pytest.fixture(scope="session")
def nlp(P):
    from shot_mpc.shot_nlp import ShotNLP

    return ShotNLP(P, jit=False, verbose=False)  # interpreted: no compile time in the tests


def rollout(physics, state, steps, r1_force=(0.0, 0.0, 0.0), stop_on_goal=False):
    """Run the simulator with a constant robot-1 force; returns per-step records."""
    u = jnp.asarray(np.concatenate([r1_force, np.zeros(3)]), dtype=jnp.float32)
    s = jnp.asarray(state, dtype=jnp.float32)
    states, scored, bounces, r1_col = [np.asarray(state, dtype=float)], [], [], []
    for _ in range(steps):
        s, c1, _, has_scored, _, wall_bounces = physics_step(s, u, physics, DT)
        states.append(np.asarray(s, dtype=float))
        scored.append(int(has_scored))
        bounces.append(np.asarray(wall_bounces))
        r1_col.append(bool(c1))
        if stop_on_goal and has_scored:
            break
    return np.array(states), np.array(scored), np.array(bounces), np.array(r1_col)
