# Puck identification

Identifies a 2D model of the air-hockey puck (free flight on the table, rebound on the walls)
from a recording of the puck EKF and the OptiTrack poses, in three steps: automatic
segmentation, manual review, fit. A viewer then compares the measured motion with the model
predictions.

## Setup

Runs on the host (it needs a display for the review and plot windows) with
[uv](https://docs.astral.sh/uv/), which creates the environment from
[pyproject.toml](pyproject.toml) and `uv.lock` on the first `uv run`:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh   # once, if uv is not installed
cd src/holohover_simulator/identification
uv sync
```

## Recording

With the mocap and the puck EKF running (`navigation` node in the `/puck` namespace), record
the EKF state, the raw puck pose and the table pose:

```bash
ros2 bag record -s mcap -o puck_id_$(date +%Y-%m-%d_%H-%M-%S) \
  /puck/state /optitrack/puck/pose /optitrack/table/pose
```

- `/puck/state` (EKF: position, yaw, velocities, yaw rate) is the data of the fit and the plots
  (`source: state` in [config.yaml](config.yaml)).
- `/optitrack/puck/pose` is used to find the segments and the walls, and to reject the EKF yaw
  where the mocap orientation is wrong. Without it everything runs on the EKF state, with worse
  impact detection (see below).
- `/optitrack/table/pose` defines the table frame used everywhere. Without it the mocap frame
  is used and the walls must be roughly aligned with its axes.

`source: pose` fits the raw pose instead (velocities then come from smoothing the positions).

### What the EKF state does at an impact

`/puck/state` is the EKF prediction at the 100 Hz timer, with a constant-acceleration model.
At every wall impact, on the first recording:
- the position keeps going into the wall for 2-3 samples (up to ~1 cm) before the next mocap
  update pulls it back,
- the velocity overshoots by up to ~35% (e.g. 1.18 instead of 0.85 m/s) and settles in ~0.1 s,
- the yaw follows the mocap orientation flips (yaw rates of hundreds of rad/s).

So the fit leaves out the EKF positions for `position_settle` (0.03 s) and the velocities for
`velocity_settle` (0.12 s) after each impact and hit, and the segmentation runs on the raw pose.
Away from contacts the EKF position matches the raw pose to ~0.25 mm.

Useful data: free glides at different speeds and directions, and bounces on the walls at
different angles and with different spins. Hits by a hand or a robot are detected and left out.

## Workflow

```bash
cd src/holohover_simulator/identification
uv run identify.py segment ../../../docker/data/puck_id_...   # automatic segments -> segments.yaml
uv run identify.py review  ../../../docker/data/puck_id_...   # check and correct them, save with s
uv run identify.py fit     ../../../docker/data/puck_id_...   # fit -> results.yaml
uv run identify.py plot    ../../../docker/data/puck_id_...   # measured vs model (windows)
uv run identify.py plot    ../../../docker/data/puck_id_... --save   # same as PNGs in plots/
```

The bag path is the bag directory or its `.mcap`. The outputs go next to the bag, or to
`results/<bag name>/` when the bag directory is not writable (bags recorded in the docker
container belong to root). `segment` does not overwrite reviewed segments without `--force`.
Settings are in [config.yaml](config.yaml); `--config my.yaml` merges a partial file over it.

### segment

Events are peaks of the smoothed acceleration. Lines fitted to the position before and after
each event give the velocities, the impact time and the contact point. An event is a **wall
impact** when the velocity change is along a table axis and reverses the normal velocity; the
walls are lines fitted to the contact points (at least 3 impacts, no data beyond them). Other
events are **hits** (hand, robot), events without velocity change are mocap **glitches**
(ignored). Data near hits, in gaps, at low speed or with a mostly invalid orientation is not
used. Each wall impact gets a `wall_impact` segment (up to 0.3 s on each side), what is left
in between becomes `glide` segments.

### review

| input | action |
|---|---|
| drag on a time plot | new segment with the current label; dragging over the selected segment changes its bounds |
| click | select a segment (shown on the table view, with warnings, e.g. it contains a hit) and move the time slider there |
| time slider (bottom) | red dot where the puck is on the table at that time, with its last 0.5 s, and a red line on the time plots |
| `g` / `w` | label glide / wall_impact (also relabels the selected segment) |
| `d`, `delete` | delete the selected segment |
| `n` / `b` | next / previous segment, zoomed |
| `a` | whole recording |
| `u` | undo |
| `s` | save |
| `q` | quit (twice with unsaved changes) |

The toolbar zoom/pan work as usual; segments are not edited while one of them is active.

### fit

Output-error fit (`scipy.optimize.least_squares`, robust loss): every segment is simulated
from its own initial state (6 unknowns per segment) and compared with the measured position,
yaw and, with the EKF state, velocity and yaw rate (except right after contacts, see above);
all segments share the model parameters and the wall offsets. The initial states are
fitted first with the model fixed, then everything together. `results.yaml` has the
parameters with their 1-sigma, the walls, and the error and fitted initial state per segment.

### plot

- **segments**: per segment, measured (dots) and model simulated from the fitted initial
  state (red) for x, y, vx, vy, yaw, yaw rate, the position error and the path on the table.
  `n` / `b` change segment, `h` adds predictions (blue) started every 0.05 s from the
  measured state, i.e. what an online predictor (the MPC) would get.
- **timeline**: the whole recording with those predictions over 0.5 s; hits are marked.
- **summary**: restitution of every impact against its position on the wall, the tangential
  impact law against every impact, glide accelerations, spin decay, prediction error against
  the horizon, error per segment.

The measured state of a prediction is the last EKF state (`source: state`) or, for the raw
pose, line fits over the last 0.06 s; predictions starting within 0.07 s after a contact are
skipped. On the first recording the EKF velocity is noisier than such a line fit of the raw
pose (prediction error at 0.5 s: ~20 mm RMS from the EKF state, ~11 mm from the line fit).

## Model

State `(x, y, yaw, vx, vy, w)` in the table frame. Equations in [model.py](model.py).

**Free flight**: `dv/dt = g - c_v v` and `dw/dt = -c_w w - c_wc sign(w)`. `g = (g_x, g_y)` is
the table tilt: on the first recording every glide had about -0.015 m/s^2 along the table
whatever its direction, a tilt of ~0.1 deg. Both have closed forms, the wall contacts are
solved exactly. A Coulomb drag on the linear motion was tried and dropped: on the data it was
not separable from `c_v` (its estimate was negative).

**Wall impact**: 2D version of the independent friction-restitution model of A.
Domenech-Carbo, *On the friction/tangential restitution problem: Independent
friction-restitution modeling of sphere rebound with arbitrary spin*, Powder Technology 2022,
[doi:10.1016/j.powtec.2022.118141](https://doi.org/10.1016/j.powtec.2022.118141): normal
restitution `e_n`, a Coulomb friction impulse (`mu`) that can at most stop the slip of the
contact point, and a tangential restitution `e_t` returning a fraction of it. With the slip
`u_t = v_t - r w`:

```
v_n' = -e_n v_n
u_t' = u_t - sign(u_t) (1 + e_t) min(|u_t|, mu (1 + e_n) |v_n| (1 + kappa) / kappa)
```

so `u_t' = -e_t u_t` when the contact sticks (small `|u_t| / |v_n|`) and the Coulomb limit when
it slides. The change of `u_t` is split between `v_t` and `w` by `kappa = I / (m r^2)` (0.5 for a
disk); the masses cancel. The summary plot shows this curve against the measured impacts. Note:
the paper's full text was not available when this was written, the formulation follows its
abstract and graphical abstract (friction slider and tangential-restitution spring in series)
and should be checked against the paper.

When the tilt pushes the puck against a wall, bounces slower than 5 mm/s become sliding along
the wall.

| parameter | unit | meaning |
|---|---|---|
| `c_v` | 1/s | linear viscous drag |
| `g_x`, `g_y` | m/s^2 | table tilt |
| `c_w`, `c_wc` | 1/s, rad/s^2 | spin viscous and Coulomb drag |
| `e_n` | - | normal restitution |
| `mu` | - | wall friction |
| `e_t` | - | tangential restitution |

`puck.radius` and `puck.kappa` in the config are not fitted.

## Using the results in the simulators

Both simulators model less than this:

- **python-sim** (`python-sim/config/config.json`): `table.friction` is the viscous drag of
  `vel *= 1 - friction dt`, i.e. `c_v`. `shot_mpc.restitution` is `e_n`. No tilt, no spin, no
  wall friction.
- **C++ simulator** (`src/simulator_node.cpp`, Box2D): puck `linearDamping` ~ `c_v`,
  `angularDamping` ~ `c_w`, `restitution` ~ `e_n`, `friction` ~ `mu`. Hardcoded at the moment
  (0.4, 0.5, 0.8, 0.02). No tilt, and Box2D has no tangential restitution.

## Tests

```bash
uv run pytest
```

`tests/test_model.py` checks the closed forms against numerical integration and the impact law
(stick, slide, energy). `tests/test_pipeline.py` writes a synthetic bag with known parameters
with the EKF state and the raw pose (`uv run identify.py synth OUT` writes one too), runs segment
and fit on both sources and checks the parameters are recovered, and drives the review and plot
code headless. The synthetic EKF state has noise but no filter dynamics.
