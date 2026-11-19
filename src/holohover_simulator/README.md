# Holohover Simulator

This repository contains the node for simulating the Holohover system, allowing for rapid development, testing, and validation of algorithms in a controlled virtual environment.

## Overview
The simulator leverages the **Box2D physics engine** to model the dynamics of the hovercraft and their interactions between each other with the environment. It simulates impacts between the hovercraft or with the table. It also simulates air drag and forces induced by table tilt.

## Nodes
- `simulator` - the Simulator node.

  The node subscribes to the `control` topic of each simulated hovercraft, and publishes its `/optitrack/*_pose_raw`.

  If the experiment includes both simulated and real hovercraft, the simulator subscribes to the `/optitrack/table_pose_raw` topic.

  Otherwise if the experiment includes only simulated hovercraft, the simulator publishes to the `/optitrack/table_pose_raw` a fixed dummy position of the table that is defined in the config file.

  It also simulates the puck (published on `/optitrack/puck_pose_raw`) and publishes `wall_collided` / `hovercraft_collided` (`std_msgs/Bool`) when a puck contact starts and ends.

  `~/reset` (`std_msgs/Float64MultiArray`) teleports everything: `(x, y, yaw, v_x, v_y, w_z)` of every simulated hovercraft, then `(x, y, v_x, v_y[, w_z])` of the puck (spin 0 when left out).

## Puck collision models

The puck's bounce on the walls and on the hovercraft is chosen separately with the node parameters `wall_collision_model` and `hovercraft_collision_model` ([src/simulator_collisions.hpp](src/simulator_collisions.hpp)):

| Model | Bounce |
|---|---|
| `box2d` | Box2D's own contact solver with the fixture values |
| `elastic` | `v_n' = -e v_n`, tangential velocity and spin unchanged |
| `walton` | normal restitution `e_n`, Coulomb friction `mu`, tangential restitution `e_t`: couples the puck's spin and tangential velocity. The model fitted by [identification/](identification/README.md) |

The coefficients are in [config/elastic.yaml](config/elastic.yaml) and [config/walton.yaml](config/walton.yaml). The wall ones are fitted on `puck_id_2026-10-13_18-15-39`; the hovercraft ones are not identified yet.

From the air hockey launch:

```bash
ros2 launch holohover_utils air_hockey_sim.launch.py wall_collision:=elastic hovercraft_collision:=box2d
```

(defaults: `wall_collision:=walton`, `hovercraft_collision:=box2d`).

## Comparing the collision models with recorded impacts

`collision_replay` runs the same Box2D puck, walls and collision models without ROS. [scripts/impact_comparison.py](scripts/impact_comparison.py) uses it to replay every wall impact of an identification recording (`results.yaml` from `identify.py fit`) with each model and writes the measured and simulated pucks to a bag:

```bash
ros2 run holohover_simulator impact_comparison.py /root/data/puck_id_2026-10-13_18-15-39
```

Output: `<bag>/impact_comparison/`, open it in Foxglove with [gui/impact_comparison.foxglove.json](gui/impact_comparison.foxglove.json).

On `puck_id_2026-10-13_18-15-39` (38 impacts), mean errors after the impact:

| | position RMS [mm] | rebound direction [deg] | rebound speed [%] | rebound spin [rad/s] |
|---|---|---|---|---|
| `box2d` | 15.6 | 3.8 | 6.8 | 2.77 |
| `elastic` | 14.3 | 5.7 | 5.8 | 4.82 |
| `walton` | 3.7 | 0.9 | 4.2 | 0.26 |

The rebound speed barely separates `elastic` and `walton`: both use the same `e_n`, and the friction mostly turns the rebound and changes the spin, not the speed. The spin and direction errors are where the models differ.

## Puck identification

[identification/](identification/README.md) fits the puck's free flight (drag, table tilt) and its wall impacts from a mocap recording.

## Python simulation

`python-sim` is a standalone Python simulator of the air-hockey table. Robot 1 is driven by the
DIAL planner or the shot MPC. A run is shown with matplotlib or exported to MCAP for Foxglove.
See [python-sim/README.md](python-sim/README.md).

```bash
cd python-sim
python main.py simulate --controller shot_mpc     # or dial
```
