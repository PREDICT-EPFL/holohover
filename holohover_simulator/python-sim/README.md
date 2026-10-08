# Holohover python simulation

A standalone air-hockey simulator in Python (JAX physics). Robot 1, on the right, attacks the left
goal. Robot 2, on the left, either stays parked in a corner or holds its home position with a PD
controller.

Robot 1 is driven by one of two controllers:

| Controller | What it does | Code |
|---|---|---|
| `dial` | Sampling-based path planner (DIAL, Till's project) plus input interpolation | [`controllers/dial/`](controllers/dial/) |
| `shot_mpc` | Gradient-based shot MPC (CasADi + IPOPT) plus a tracking QP | [`controllers/shot_mpc/`](controllers/shot_mpc/), see [`docs/air-hockey-mpc-formulation.md`](docs/air-hockey-mpc-formulation.md) |

## Layout

```
main.py                CLI: simulate | evaluate | animate | mcap
config/config.json     the only config: physics, scenarios, evaluation, and one section per controller
simulation/
  dynamics.py          physics_step: robots, puck, walls, goals
  setup.py             config loading, physics parameters, initial states
  episode.py           the closed loop shared by both controllers, and its recorded data
controllers/
  __init__.py          the controller interface, and load() to pick one
  pd.py                PD to home (defence, robot 2), DIAL's imagined opponent
  dial/                controller.py, planner.py, rollout.py
  shot_mpc/            see its __init__.py
viz/
  animation.py         matplotlib window or GIF
  foxglove_viz.py      MCAP file for the Foxglove app
  foxglove_layout.json Foxglove layout
tests/                 pytest
```

Each control step (`simulation.hz`, 100 Hz) works in the same way for both controllers:

1. Robot 1's controller computes its input.
2. Robot 2 gets its own input (PD to home, or none when it is parked).
3. The physics runs `simulation.physics_hz / hz` substeps under those inputs.

Every step is recorded in one data dict, which both viewers read (`simulation.episode.episode_data`).

## Run

From `python-sim/`, either in the devcontainer or with `uv sync` and then `.venv/bin/python`:

```bash
python main.py simulate                                   # shot_mpc (the config's), scenario "default", animated
python main.py simulate --controller dial --scenario incoming
python main.py simulate --scenario stationary --mcap --no-animate
python main.py evaluate --episodes 50 --seed 0            # random starts: conversion rate
python main.py evaluate --mcap --outcomes miss no_contact # also record the failures for Foxglove
python main.py animate                                    # the newest saved run
python main.py animate simulation_data/<run>.pkl --mode dial   # step through DIAL's samples (keys)
python main.py mcap simulation_data/<run>.pkl             # Foxglove file of a saved run
```

Options shared by `simulate` and `evaluate`:
- `--controller dial|shot_mpc`
- `--opponent none|static`
- `--config PATH`
- `--mcap`
- `--gif NAME`, which writes `gifs/NAME.gif` instead of opening a window

Options for `shot_mpc` only:
- `--workers N`: number of solver processes, 0 for serial solves
- `--no-jit`
- `--no-tracker`

Outputs (both folders are git-ignored):
- `simulation_data/<controller>_<scenario>_<stamp>.pkl`, the recorded data;
- `<...>_log.json`, with the outcome, the contacts and the controller's log;
- `<...>.mcap` with `--mcap`;
- GIFs in `gifs/`.

Tests: `python -m pytest` (`-m "not slow"` skips the closed-loop run).

## Config

`config/config.json` holds:

| Key | Contents |
|---|---|
| `controller` | the default controller |
| `simulation` | control and physics rates, duration, force limits, `opponent` (`none` or `static`) |
| `table`, `robot`, `puck` | the physical parameters; robot 2 is parked at `robot.robot2.park` when `opponent` is `none` |
| `scenarios` | named starts for `simulate --scenario` |
| `evaluation` | how `evaluate` draws its random starts |
| `dial`, `shot_mpc` | the controllers' parameters |

## Foxglove

1. Write an MCAP: `simulate --mcap`, `evaluate --mcap` or `mcap <pkl>`.
2. Go to <https://app.foxglove.dev>, choose **Open local file…** and pick the `.mcap`.
3. Import the layout [`viz/foxglove_layout.json`](viz/foxglove_layout.json) from the layout menu (**Import from file…**).

The topics are listed at the top of [`viz/foxglove_viz.py`](viz/foxglove_viz.py). The layout shows:
- a top-down 3D view with the plan, the predictions, the puck trail, and either the shot MPC's bounce-sequence candidates or DIAL's samples (`/dial/samples`);
- velocity and force plots;
- the event log.

For `shot_mpc` only, the layout also shows:
- the sequence scores;
- the selector update times;
- a **Compute** timeline: one row for the selector and one per sequence, each bar as wide as the wall time of the solve.

If you change the layout in the app, export it over `viz/foxglove_layout.json`.

## Notes on DIAL

- `dial.H_interpolation` (10) differs from `dial.H` (15). The interpolation controller therefore plays a 15-node plan as if it had 10 nodes, i.e. 1.5× slower than planned. This is kept as it was in Till's code.
- DIAL used to run with robot 2 at home (`--opponent static`). The config default is `none`.
- Physics now runs at 1000 Hz for both controllers. DIAL used to step the physics at 100 Hz.
