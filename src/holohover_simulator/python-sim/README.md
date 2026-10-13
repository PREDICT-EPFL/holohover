# Holohover python simulation

A standalone air-hockey simulator in Python (JAX physics). Robot 1, on the right, attacks the left
goal. Robot 2, on the left, either stays parked in a corner or holds its home position with a PD
controller.

Robot 1 is driven by one of these controllers:

| Controller | What it does | Code |
|---|---|---|
| `dial` | Sampling-based path planner (DIAL, Till's project) plus an MPC path follower | [`controllers/dial/`](controllers/dial/), see `SP_hockey/literature/report-till.pdf` |
| `shot_mpc` | Gradient-based shot MPC (CasADi + IPOPT) plus a tracking QP | [`controllers/shot_mpc/`](controllers/shot_mpc/), see [`docs/air-hockey-mpc-formulation.md`](docs/air-hockey-mpc-formulation.md) |
| `forward` | Example: a constant force toward the opponent goal (`forward.force`), a template for a new controller | [`controllers/forward/`](controllers/forward/) |

## Layout

```
main.py                CLI: simulate | evaluate | view | starts
config/config.json     the only config: physics, scenarios, evaluation, and one section per controller
config/eval_starts.json the 1000 random starts evaluate runs
simulation/
  dynamics.py          physics_step: robots, puck, walls, goals
  setup.py             config loading, physics parameters, initial states
  episode.py           the closed loop shared by all controllers, and its recorded data
controllers/
  __init__.py          the controller interface, and load() to pick one
  pd.py                PD to home (shot_mpc defence, robot 2)
  dial/                controller.py, planner.py, rollout.py, follower.py (MPC, LQR), model.py
  shot_mpc/            see its __init__.py
  forward/             example: constant force forward
viz/
  foxglove_viz.py      MCAP file for the Foxglove app
  foxglove_open.py     opens an MCAP file in the Foxglove web app
  view_starts.py       shows the evaluation starts in Foxglove (python viz/view_starts.py)
  foxglove_layout.json Foxglove layout
tests/                 pytest
```

Each control step (`simulation.hz`, 100 Hz) works in the same way for every controller:

1. Robot 1's controller computes its input.
2. Robot 2 gets its own input (PD to home, or none when it is parked).
3. The physics runs `simulation.physics_hz / hz` substeps under those inputs.

Robot 1's input can reach the physics late, to test a controller's robustness to delay:
`simulation.input_delay` (or `--input-delay`) is a number of control steps, 0 by default. The input
computed at step `i` is applied at step `i + delay`, and the input is zero until the first one arrives.
The controller doesn't know about the delay. The recorded `/robot1/control` is the applied input; for
`shot_mpc`, `/robot1/plan_control` and `/robot1/def_control` are still the computed ones.

`shot_mpc` can compensate a delay it assumes, `shot_mpc.delay_compensation` control steps (or
`--compensate [STEPS]`, by default the input delay). Its planner and tracker then start from the state
predicted for when the input is applied: robot 1 rolled through its pending inputs with the planner's ZOH
model, and the puck along its free flight. The defence PD also acts on the predicted robot state.

Every step is recorded in one data dict (`simulation.episode.episode_data`), which is written to an MCAP file for Foxglove.

## Run

From `python-sim/`, either in the devcontainer or with `uv sync` and then `.venv/bin/python`:

```bash
python main.py simulate                                   # shot_mpc (the config's), scenario "default", opened in Foxglove
python main.py simulate --controller dial --scenario incoming
python main.py simulate --scenario stationary --no-view   # only save the MCAP
python main.py evaluate --episodes 50                     # the first 50 stored starts: conversion rate
python main.py evaluate --mcap --outcomes miss no_contact # also save the failures, in one MCAP file
python main.py evaluate --first 37 --episodes 1 --mcap    # rerun start 37 only
python main.py view                                       # open the newest saved run in Foxglove
python main.py view simulation_data/<run>.mcap
```

Options shared by `simulate` and `evaluate`:
- `--controller dial|shot_mpc|forward`
- `--opponent none|static`
- `--input-delay STEPS`: robot 1's input is applied STEPS control steps late (`simulation.input_delay`)
- `--config PATH`

Options for `shot_mpc` only:
- `--workers N`: number of solver processes, 0 for serial solves
- `--no-tracker`
- `--compensate [STEPS]`: delay compensation over STEPS control steps, by default `--input-delay`'s

Outputs go to `simulation_data/` (git-ignored): `<controller>_<scenario>_<stamp>.mcap` for `simulate`, and
`<controller>_eval_<stamp>.mcap` for `evaluate --mcap`, which holds the saved episodes one after the other,
1 s apart. Each file has the config and the log (outcome, contacts, the controller's log) attached as
`config.json` and `log.json`; an evaluation file has one `log_ep<i>.json` per episode instead of `log.json`,
and marks each episode's start on `/episode` (the Episode row of the Mode panel) and `/events`.

Tests: `python -m pytest` (`-m "not slow"` skips the closed-loop run).

## Evaluation starts

`evaluate` doesn't sample: it runs the starts stored in `config/eval_starts.json` (config `evaluation.starts`),
so every run, controller and machine sees the same test set. Episode `i` starts from start `i`, and
`--first`/`--episodes` pick a slice of the 1000. Each start gives robot 1 and the puck as
`{"robot1": [x, y, theta, vx, vy, omega], "puck": [...]}`, like a config scenario; robot 2 comes from
the config (`--opponent`).

The file was drawn with `python main.py starts --count 1000 --seed 0` from `evaluation.sampling`, and
records that seed and those parameters. They are the starts `evaluate --seed 0` drew before the file
existed, bit for bit, so earlier seed-0 results still apply. Redrawing it (`starts --force`) makes old
results incomparable; `tests/test_starts.py` fails if the file no longer matches its seed and the config's
sampling parameters.

To look at them:

```bash
python viz/view_starts.py                       # all 1000 in Foxglove: start i at t = i s
python viz/view_starts.py --first 37 --count 10 # start 37 + k at t = k s
```

The table then shows the current start, with an arrow for the puck's first 0.5 s, over a cloud of
all the starts shown. The Events panel gives each start's index, puck speed and heading.

## Config

`config/config.json` holds:

| Key | Contents |
|---|---|
| `controller` | the default controller |
| `simulation` | control and physics rates, duration, force limits, `input_delay` (control steps), `opponent` (`none` or `static`) |
| `table`, `robot`, `puck` | the physical parameters; robot 2 is parked at `robot.robot2.park` when `opponent` is `none` |
| `scenarios` | named starts for `simulate --scenario` |
| `evaluation` | `duration` of an episode, the `starts` file, and the `sampling` that `main.py starts` draws it from |
| `dial`, `shot_mpc`, `forward` | the controllers' parameters |

## Foxglove

`simulate` and `view` open the run in the Foxglove web app. The web app can't be pointed at a local file,
so `main.py` serves the file on `127.0.0.1:8790` (or the next free port) and opens your browser on a
link of this form:

```
https://app.foxglove.dev/~/view?ds=remote-file&ds.url=http://127.0.0.1:8790/<run>.mcap
```

The link is also printed (Ctrl+click it in the VS Code terminal). The file is served until Ctrl-C.
`view --no-browser` only prints the link.

- The first time, import the layout [`viz/foxglove_layout.json`](viz/foxglove_layout.json) from the layout menu (**Import from file…**). The app then keeps using it.
- Chrome may ask to let app.foxglove.dev access apps on this device. Allow it, or the app can't read the file.
- In the devcontainer, the port must be forwarded to the host. VS Code usually does this by itself; if it doesn't, add 8790 in its **Ports** tab.
- Without the server, you can still open a `.mcap` by hand with **Open local file…** in the app.

## Notes on DIAL

Works but needs to be checked.
