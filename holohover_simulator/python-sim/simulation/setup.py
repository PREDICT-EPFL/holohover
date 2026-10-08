"""Configuration, physics parameters and initial states, shared by every controller."""

import json

import numpy as np
import jax.numpy as jnp

from .dynamics import Table, Entity, PhysicsParams


def load_config(path="config/config.json"):
    with open(path, "r") as f:
        return json.load(f)


def physics_from_config(config):
    """Table, robot and puck parameters of the simulator. Creates JAX arrays, which starts JAX's
    threads: build the controllers first (controllers.load), the shot MPC forks processes."""
    table = Table(**config["table"])
    robot = Entity(
        radius=config["robot"]["radius"],
        mass=config["robot"]["mass"],
        inertia=0.5 * config["robot"]["mass"] * config["robot"]["radius"] ** 2,
        u_limits=jnp.array(config["simulation"]["u_limits"]),
    )
    puck = Entity(
        radius=config["puck"]["radius"],
        mass=config["puck"]["mass"],
        inertia=0.5 * config["puck"]["mass"] * config["puck"]["radius"] ** 2,
        u_limits=jnp.array([0, 0, 0]),
    )
    return PhysicsParams(table=table, robot=robot, puck=puck)


def _robot2(config):
    """Robot 2 at its initial position when it defends (opponent "static"), else parked in a corner
    of its end, never actuated (opponent "none")."""
    r2 = config["robot"]["robot2"]
    if config["simulation"]["opponent"] == "static":
        return r2["initial_position"] + r2["initial_velocity"]
    return [r2["park"][0], r2["park"][1], 0.0, 0.0, 0.0, 0.0]


def initial_state(config, scenario=None):
    """18-dim state [robot1, robot2, puck], each (x, y, theta, vx, vy, omega). A scenario of the
    config replaces the initial robot 1 and puck states."""
    r1 = config["robot"]["robot1"]["initial_position"] + config["robot"]["robot1"]["initial_velocity"]
    puck = config["puck"]["initial_position"] + config["puck"]["initial_velocity"]
    if scenario:
        sc = config["scenarios"][scenario]
        r1 = sc.get("robot1", r1)
        puck = sc.get("puck", puck)
    return np.array(r1 + _robot2(config) + puck, dtype=float)


def sample_initial_state(rng, config):
    """Random start for the evaluation: robot 1 near home, puck stationary in our half or incoming."""
    ev = config["evaluation"]
    width, height = config["table"]["width"], config["table"]["height"]
    r1 = np.array([config["robot"]["robot1"]["defensive_position_rel"] * width, 0.5 * height, 0.0, 0, 0, 0])
    r1[:2] += rng.uniform(-ev["robot1_jitter"], ev["robot1_jitter"], 2)
    d_min = config["robot"]["radius"] + config["puck"]["radius"] + 0.05
    while True:
        y = rng.uniform(*ev["puck_y"])
        if rng.uniform() < ev["stationary_fraction"]:
            puck = np.array([rng.uniform(*ev["puck_x_stationary"]), y, 0.0, 0.0, 0.0, 0.0])
        else:
            speed = rng.uniform(*ev["speed"])
            heading = np.deg2rad(rng.uniform(*ev["heading_deg"]))  # around +x: toward our goal
            puck = np.array([rng.uniform(*ev["puck_x_moving"]), y, 0.0,
                             speed * np.cos(heading), speed * np.sin(heading), 0.0])
        if np.linalg.norm(puck[:2] - r1[:2]) > d_min:
            break
    return np.concatenate([r1, _robot2(config), puck])
