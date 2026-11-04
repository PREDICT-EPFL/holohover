# Holohover control
This package contains nodes to control a single hovercraft.

## Nodes
- `control_lqr` - Control a single hovercraft using an LQR controller. This node uses the disturbance estimator 
- `control_exp` - Control a single hovercraft adding some noise to a standard LQR controller. Used for System Identification.
- `control_signal` - Single motor SysId. Test-stand with a force sensor to measure each motor individually. Removed from CMakeLists to speed-up compile time.
- `control_plan` (Python) - Follows the plans (`holohover_msgs/HolohoverPlan`, its chosen candidate) of a planner such as `holohover_planner`'s `shot_planner`. With a plan (attack), a tracking QP (`holohover_control/tracker.py`, CasADi + DAQP) follows it up to the planned impact and holds the heading at `tracker.yaw_ref`; past the impact its last solve runs open loop. Without a plan, after a plan with no chosen candidate (contact) or `plan_expiry` past the impact (defence), a PD holds the hovercraft at `defence.home`. The commanded acceleration becomes motor signals with `control_lqr`'s actuation (`holohover_control/actuation.py`: holohover_common's model through its Python bindings; motor dynamics, thrust allocation QP, inverse of the motor lag, disturbance estimate). Topics: `state_disturbance`, `plan`, `episode` (evaluations: -1 idles it: no acceleration, no disturbance compensation; an episode index restarts it in defence; parameter `start_idle`) in; `control` (motor signals), `acc_ref` (commanded acceleration, table frame) out. Parameters: `holohover_utils/config/common/control_plan_config.yaml`, their only values (no defaults in the code: a missing one stops the node), and `holohover_props_file` from the launch file.

## Python controllers
The control of `control_plan` is plain Python, without ROS (`holohover_control/`): `plan_controller.py` (modes, plan expiry, defence PD), `tracker.py` (tracking QP), `plan.py` (a plan and its ZOH model). The node only feeds it. python-sim (`holohover_simulator/python-sim`, shot MPC) runs the same classes: its `uv sync` installs this folder with `pyproject.toml`, which colcon ignores. They need casadi from pip (installed by `docker/Dockerfile` with `holohover_planner`'s requirements).

```bash
colcon test --packages-select holohover_control && colcon test-result --verbose
```

## ToDos
- Set a parameter for the LQR node whether to use the disturbance estimator topic or not.
- Update the control_exp to use the disturbance estimator?