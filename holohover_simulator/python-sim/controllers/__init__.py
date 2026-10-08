"""
Robot 1 controllers, selected by the config's "controller":

    dial      sampling-based path planner (DIAL) plus input interpolation (controllers/dial)
    shot_mpc  gradient-based shot MPC plus tracking QP (controllers/shot_mpc)

A controller is built per episode by its factory, and provides
    mode                      DEF or ATT
    step(i, state) -> F       robot 1's input (Fx, Fy, tau) at control step i, world frame, clipped
    notify_contact(t)         robot 1 touched the puck at time t
    record(state, t) -> dict  what the viewers show at this step (simulation.episode.episode_data)
    log() -> dict             JSON-able episode log, saved next to the data

A factory, built once per process by load(), provides
    new(physics, init_state, verbose) -> controller
    summary(logs) -> str      report on the logs of some episodes (may be empty)
    close()
"""

DEF, ATT = 0, 1  # defend at home, attack the puck
CONTROLLERS = ("dial", "shot_mpc")


def load(config, workers=None):
    """The factory of config["controller"]. Call it before simulation.setup.physics_from_config:
    the shot MPC forks its solver processes here, before JAX starts its threads (forking a
    multithreaded process can deadlock the child). workers: shot_mpc solver processes
    (default shot_mpc.n_workers, 0: serial)."""
    name = config["controller"]
    if name == "dial":
        from .dial.controller import DialFactory
        return DialFactory(config)
    if name == "shot_mpc":
        from .shot_mpc.shot_controller import ShotFactory
        return ShotFactory(config, workers)
    raise ValueError(f"unknown controller {name!r}: use one of {', '.join(CONTROLLERS)}")
