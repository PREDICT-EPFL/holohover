"""DIAL port: planning model, LQR defence, goal check, and a closed-loop shot."""

import numpy as np
import jax.numpy as jnp
import pytest

from conftest import PUCK_PARK, R2_PARK
from controllers.dial.follower import build_lqr_controller
from controllers.dial.model import discrete_system, heading_to_goal, physics_step
from controllers.pd import home_state


def test_discrete_system_matches_model_step(physics):
    """The follower's (A_d, B_d) is the planning model's step for a free robot."""
    dt = 0.05
    x = np.array([1.5, 0.5, 0.1, 0.2, -0.1, 0.0])
    u = np.array([0.2, -0.3, 0.001])
    state = jnp.asarray(np.concatenate([x, R2_PARK, PUCK_PARK]), dtype=jnp.float32)
    next_state = physics_step(state, jnp.concatenate([jnp.asarray(u), jnp.zeros(3)]), physics, dt)[0]
    Ad, Bd = discrete_system(physics, dt)
    assert np.allclose(np.asarray(next_state[:6]), np.asarray(Ad @ x + Bd @ u), atol=1e-6)


def test_lqr_holds_home(config, physics):
    """From 20 cm off, the LQR brings robot 1 home under the plant within 3 s."""
    from simulation.dynamics import physics_step as plant_step

    home = np.array(home_state(config, 1))
    lqr = build_lqr_controller(home, config, physics)
    state = jnp.asarray(np.concatenate([home + [-0.2, 0.15, 0, 0, 0, 0], R2_PARK, PUCK_PARK]), dtype=jnp.float32)
    for _ in range(300):
        u = jnp.clip(lqr(state), -physics.robot.u_limits, physics.robot.u_limits)
        state = plant_step(state, jnp.concatenate([u, jnp.zeros(3)]), physics, 0.01)[0]
    assert np.linalg.norm(np.asarray(state[:2]) - home[:2]) < 0.01


@pytest.mark.parametrize("vel, expected", [
    ([-1.0, 0.0], True),  # straight at the goal
    ([-1.0, 0.45], False),  # above the goal
    ([1.0, 0.0], False),  # away from it
])
def test_heading_to_goal(physics, vel, expected):
    assert bool(heading_to_goal(physics, jnp.array([1.0, 0.53]), jnp.array(vel))) == expected


@pytest.mark.slow
def test_closed_loop_scores(config, physics):
    import controllers
    from simulation.episode import run_episode
    from simulation.setup import initial_state

    config = {**config, "controller": "dial"}
    factory = controllers.load(config)
    init = initial_state(config, "stationary")
    res = run_episode(config, physics, factory.new(physics, init), init, 6.0, stop_on_goal=True, record=False)
    assert res["outcome"] == "goal", res["outcome"]
