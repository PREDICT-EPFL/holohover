# holohover_utils

Launch files, experiment configs, interface nodes, GUI layouts and small tools. Everything runs in the
docker container (`./holohover shell`). From the host, `./holohover launch <package> <file> [args]` does the
same as `ros2 launch`.

## Launchers

Most launchers take `experiment:=<file>` (a file in [config/experiments/](config/experiments/)) and
`machine:=<name>` (default `master`). Each node runs on the machine the experiment file names for it, and
`machine:=all` starts everything on this machine. `hw_env` and `mult.*` default to `experiment1.yaml`, which
does not exist, so always pass them an experiment.

### Air hockey (Box2D simulation)

| Launcher | What it starts |
|---|---|
| [air_hockey_sim.launch.py](launch/air_hockey_sim.launch.py) | The whole game: the simulator (table + puck), the optitrack interface, the puck's EKF and, per hovercraft, its EKF + `shot_planner` + `control_plan`. With `controller:=teleop`, `control_lqr` + teleop (gamepad or keyboard) instead. |

```bash
ros2 launch holohover_utils air_hockey_sim.launch.py                         # play
ros2 launch holohover_utils air_hockey_sim.launch.py controller:=teleop      # drive h0 yourself
ros2 launch holohover_utils air_hockey_sim.launch.py evaluate:=50 first:=0   # score 50 python-sim starts, then stop
```

| Argument | Default | |
|---|---|---|
| `experiment` | `air_hockey.yaml` | experiment file |
| `controller` | `plan` | `plan` (shot planner) or `teleop` (`hN/cmd_vel`; gamepad: hold L1) |
| `puck` | experiment's | `x,y` of the puck at rest, table frame |
| `wall_collision` | `elastic` | puck-wall bounce: `box2d`, `elastic`, `friction_restitution` (see holohover_simulator) |
| `hovercraft_collision` | `box2d` | puck-hovercraft bounce, same options |
| `substeps` | `10` | Box2D steps per simulator period |
| `record` | `false` | record every topic to `/root/data/rosbags/air_hockey_<date>_<time>` |
| `evaluate`, `first` | `0`, `0` | run `air_hockey_evaluate.py` on that many python-sim starts; results in `/root/data/box2d_eval` |
| `python_sim` | next to the source | python-sim's directory (for `evaluate`) |

View it in Foxglove (`ws://localhost:8765`, or a recorded bag) with [gui/air_hockey.foxglove.json](gui/air_hockey.foxglove.json).

### Experiments (real and/or simulated hovercraft)

These are split into two launches: the common nodes once, and the hovercraft stack on each machine.

| Launcher | What it starts |
|---|---|
| [hw_env.launch.py](launch/hw_env.launch.py) | Common nodes: the optitrack interface, the simulator (if any hovercraft in the experiment is simulated), RViz (`rviz:=false` to skip) and the recorder (`record:=true`). |
| [mult.lqr.simulation.launch.py](launch/mult.lqr.simulation.launch.py) | Per hovercraft: EKF + LQR controller (`hovercraft_lqr.launch.py`). |
| [mult.dmpc.simulation.launch.py](launch/mult.dmpc.simulation.launch.py) | Distributed MPC: `trajectory_generator` + `dmpc_trigger`, DMPC per hovercraft (`hovercraft_dmpc.launch.py`), LQR per obstacle, and the recorder. |
| [simulation_base.launch.py](launch/simulation_base.launch.py) | Self-contained (older): simulator, rviz/optitrack interfaces, RViz and LQR per hovercraft. |
| [simulation_mpc_puck.launch.py](launch/simulation_mpc_puck.launch.py) | Copy of `simulation_base` with `holohover_playground`'s MPC (`hovercraft_mpc_play.launch.py`). Hard-coded for 2 hovercraft. |

```bash
ros2 launch holohover_utils hw_env.launch.py experiment:=hw_table_frame.yaml
ros2 launch holohover_utils mult.lqr.simulation.launch.py experiment:=hw_table_frame.yaml   # second terminal
```

### Building blocks (included by the ones above, rarely launched directly)

| Launcher | What it starts |
|---|---|
| [hovercraft_lqr.launch.py](launch/hovercraft_lqr.launch.py) | One hovercraft: `navigation_disturbance` (EKF) + `control_lqr` |
| [hovercraft_mpc.launch.py](launch/hovercraft_mpc.launch.py) | One hovercraft: `navigation` + `holohover_mpc` `control_mpc` |
| [hovercraft_mpc_play.launch.py](launch/hovercraft_mpc_play.launch.py) | One hovercraft: `navigation` + `holohover_playground` `control_mpc` |
| [hovercraft_dmpc.launch.py](launch/hovercraft_dmpc.launch.py) | One hovercraft: `navigation_disturbance` + `control_dmpc_<alg>`, records its own topics |
| [hovercraft_exp.launch.py](launch/hovercraft_exp.launch.py) | One hovercraft: `control_exp` only (random motor signals, `control_exp_config.yaml`) |
| [common/optitrack.launch.py](launch/common/optitrack.launch.py) | `mocap_optitrack` node (real OptiTrack, [config/optitrack/](config/optitrack/)) |
| [common/micro_agent.launch.py](launch/common/micro_agent.launch.py) | micro-ROS agent, UDP port 8888 (talks to the hovercraft firmware) |
| [common/embedded_controller.launch.py](launch/common/embedded_controller.launch.py) | On the hovercraft: `holohover_fc` + `holohover_mouse_sensor` drivers |
| [common/rviz.launch.py](launch/common/rviz.launch.py) | RViz + `rviz_interface` |
| [common/recorder.launch.py](launch/common/recorder.launch.py) | MCAP bag of every topic except `/hN/...` (`record:=true`, `record_delay:=<s>`) |
| [common/recorder_specific.launch.py](launch/common/recorder_specific.launch.py) | MCAP bag of one hovercraft's `/<name>/...` topics |

[experiment_config_parser.py](launch/experiment_config_parser.py) is the experiment file reader the launchers share.

## Nodes and scripts

| Run | What it does |
|---|---|
| `ros2 run holohover_utils optitrack_interface` | Raw OptiTrack poses (`/optitrack/*_pose_raw`) to the table frame |
| `ros2 run holohover_utils rviz_interface` | Hovercraft and puck markers for RViz |
| `ros2 run holohover_utils air_hockey_evaluate.py` | Scores the air-hockey nodes on python-sim's starts (started by `air_hockey_sim evaluate:=N`) |
| `ros2 run holohover_utils ping_estimator.py` | Round-trip time over `/drone/ping` / `/drone/pong` |
| [scripts/bag_window.py](scripts/bag_window.py) | Host: print a bag's messages around a time, or cut out one evaluation episode (see its docstring) |
| [scripts/mcap_to_csv.py](scripts/mcap_to_csv.py) | Host: MCAP bag to CSV |

## Config and GUI

- [config/common/](config/common/): the parameters of every node (simulation, navigation, controllers, planner, drivers, teleop).
- [config/experiments/](config/experiments/): which hovercraft and obstacles take part, where they start, whether each one is simulated, and on which machine.
- [gui/](gui/): the Foxglove layout for air hockey, plus RViz configs and rqt perspectives.
