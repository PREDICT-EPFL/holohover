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
