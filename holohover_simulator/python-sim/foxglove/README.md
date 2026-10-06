# Foxglove visualisation

Simulation runs are written to MCAP files with Foxglove message types (topics listed in
[`foxglove_viz.py`](../foxglove_viz.py)) and viewed in the Foxglove web app.

## Write an MCAP

Run from `python-sim/`:

```bash
# one episode: simulation_data/shot_incoming_<stamp>.mcap next to the .pkl
uv run python run_shot.py --scenario incoming --mcap --no-animate

# evaluation: one simulation_data/shot_eval_<stamp>_ep<i>_<outcome>.mcap per recorded episode
uv run python run_shot.py --evaluate --episodes 20 --mcap --export-outcomes miss no_contact

# convert an existing pickle (run_shot.py or main.py); uses its _log.json when present
uv run python foxglove_viz.py simulation_data/shot_incoming_<stamp>.pkl
```

## Open it

1. Go to <https://app.foxglove.dev> and choose **Open local file…**, then pick the `.mcap`.
2. Import the layout from the layout menu in the top bar (**Import from file…**) and choose
   `foxglove/shot_mpc_layout.json`.

The layout has:

- a top-down 3D view of the table with the plan, predictions, puck trail and bounce-sequence
  candidates;
- plots of robot 1 and puck velocity, robot 1 force, the sequence scores and selector update times;
- mode and chosen sequence over time, the event log, and the last selector update.

In the 3D panel, each candidate is its own topic (`/shot_mpc/candidates/<label>`), so you can hide
sequences one by one. Slowing the playback down (speed control in the bottom bar) helps when looking
at impacts.

If you change the layout in the app, export it over `shot_mpc_layout.json` so everyone gets the change.

## Live view

[`run_live.py`](../run_live.py) runs the same closed loop as `run_shot.py` and streams it to the app
over a WebSocket while it runs, at a speed you choose:

```bash
uv run python run_live.py                                  # random starts, half real time
uv run python run_live.py --scenario incoming --speed 0.25
uv run python run_live.py --auto-restart                   # next episode after each goal
```

1. Open the link it prints (or **Open connection…** > Foxglove WebSocket > `ws://localhost:8765`).
   The simulation starts once the app subscribes.
2. Import `foxglove/shot_mpc_live_layout.json`: the offline layout plus a control row above the table.

Controls:

- **Pause, Resume, Step 10 ms, Step 50 ms** (the `/pause` and `/step` services): a step pauses the
  simulation if it is running, then advances one control step (10 ms) or one planning tick (50 ms).
  While paused, the app keeps showing the last step, so you can look at it in every panel.
- **Save** (the `/save` service): writes the current episode, from its start to now, to
  `simulation_data/live_<start>_<date>_<time>.mcap`. Open it with **Open local file…** to seek and
  step through the shots, with either layout (on a file, the live layout's buttons do nothing). Every
  episode is recorded to a temporary file while it runs, deleted when the next one starts or when
  `run_live.py` stops, so only what you save is kept. Saving again later in the same episode gives a
  new, longer file. The path is printed in the terminal and in the event log.
- **Restart, Random, Stationary, Incoming, Around** (the `/reset` service): Restart repeats the current
  start; the others switch to that start. From a Call Service panel you can also send
  `{"scenario": "random", "seed": 3}`. Random draw k with seed s is episode k of
  `run_shot.py --evaluate --seed s`.
- **Live controls** (Parameters panel): `speed` as a fraction of real time, `paused` and
  `auto_restart`. Type the new JSON value, then click the check mark to send it. By default an
  episode goes on until you press Restart or another start, through goals and past its duration.
  With `auto_restart` (or `--auto-restart`), an episode ends one second after the first goal or at
  its duration, and the next one starts: the same scenario, or the next random draw.
- **Speed** plot: requested against achieved speed. The control-step compute time is in the
  Compute time plot.

The simulated time does not depend on the wall clock, so a live episode is the same as the offline
one. An outer update solves the five bounce sequences in parallel, one worker process each
(`shot_mpc.n_workers` in the config, `--workers N` on the command line, `--workers 0` to solve them
serially). The first step of an episode then takes about 30 ms (at most ~0.1 s), and the pacing makes
up for it: at speed 1 the achieved speed stays about 1.0. A step more than 0.1 s behind schedule makes
the view wait for it rather than fast-forward afterwards, so the achieved speed drops below the
requested one.

Inside the devcontainer, start it with `--host 0.0.0.0` and forward port 8765.
