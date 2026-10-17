# Holohover planner
Planners of the holohover. A planner plans the motion of one hovercraft and publishes it as a
`holohover_msgs/HolohoverPlan`; a controller follows the plan.

## Nodes
- `shot_planner` - Shot MPC planner (bilevel gradient-based, CasADi + IPOPT; formulation in python-sim's
  `docs/air-hockey-mpc-formulation.md`). It plans the hovercraft's path up to its impact with the puck
  so that the puck goes into the attacked goal, directly or off at most `K` side walls.

### shot_planner
| Topic | Type | |
|---|---|---|
| `state` (in) | `holohover_msgs/HolohoverStateStamped` | the hovercraft's EKF (`holohover_navigation`), table frame |
| `/puck/state` (in) | `holohover_msgs/HolohoverStateStamped` | the puck's EKF, table frame (`holohover_navigation` `navigation` in namespace `puck`, started by `holohover_utils` `launch/air_hockey_sim.launch.py`) |
| `episode` (in) | `std_msgs/Int32` | evaluations (`holohover_utils` `air_hockey_evaluate.py`): -1 idles the planner, an episode index restarts it afresh; parameter `start_idle` |
| `plan` (out) | `holohover_msgs/HolohoverPlan` | after every planning update: one `HolohoverPlanCandidate` per bounce sequence (direct, L, R, LR, RL), with its cost J*(B) and its N+1 states and N accelerations up to the impact (table frame, from its `t0` over `tf`), empty (cost NaN) when the sequence has no valid plan; `chosen`: index of the plan to follow, -1 for none (every candidate empty after a contact or an expired plan) |

Parameters: `holohover_utils/config/common/shot_planner_config.yaml`, their only values: the code has no
defaults, and a missing parameter stops the node (`start_idle` excepted, set by the launch file).

```bash
ros2 run holohover_planner shot_planner --ros-args -r __ns:=/h0 \
    --params-file $(ros2 pkg prefix holohover_utils)/share/holohover_utils/config/common/shot_planner_config.yaml
```

How it runs:
- **Rates.** A timer at `shot_mpc.rates.inner_hz` (20 Hz) re-solves the chosen bounce sequence; every
  `1 / outer_hz` (5 Hz) all sequences are solved and the cheapest valid one is chosen. Without a plan, only
  the 5 Hz updates run. Near the impact the sequence is committed (`t_commit`); below `t_stop_replan` the plan
  is no longer re-solved; a plan whose impact passed by `plan_expiry` is dropped.
- **Timing.** A plan starts at its tick's time (its candidate's `t0`), from the robot and puck states advanced
  to it with the planner's models. It is published once solved (up to some 40 ms later): the controller
  follows it on the clock.
- **Solver processes.** The sequences of a 5 Hz update are solved in parallel in `shot_mpc.n_workers`
  spawned processes (IPOPT/MUMPS does not run in parallel threads), bit-identical to serial solves.
- **Contacts.** Detected with the planner's puck model (`contact.py`): a puck velocity the free flight with
  wall reflections does not explain, with the robot next to the puck. A contact drops the plan and the warm
  starts, publishes a plan with every candidate empty and pauses planning for `contact_cooldown`.
- **Candidates.** A candidate is its sequence's latest valid solve: at an inner update only the chosen
  sequence is re-solved, the others keep their result (and `t0`) from the last outer update. The chosen
  candidate is the plan being followed, which stays chosen when its latest re-solve fails.
- **Heading.** Plans carry none (yaw = w_z = 0): the controller holds the heading.
- **Last input.** The NLP's input-rate cost uses the current plan's input as the last applied one (the
  applied input is not known to the planner).

## Code
The planning is plain Python, without ROS: `shot_planner.py` (`ShotPlanner`: ticks, commit, expiry, contacts;
table frame in and out) on top of `shot_mpc/`; `contact.py` detects contacts from the puck's EKF. The node
(`shot_planner_node.py`) only feeds `ShotPlanner` and publishes its plans. python-sim
(`holohover_simulator/python-sim`, `controllers/shot_mpc`) runs the same `ShotPlanner`, with the simulator's
contacts and the applied input as the last input: its `uv sync` installs this package from `src/`.

## Tests
```bash
colcon test --packages-select holohover_planner && colcon test-result --verbose
```
