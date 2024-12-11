# Holohover control
This package contains nodes to control a single hovercraft.

## Nodes
- `control_lqr` - Control a single hovercraft using an LQR controller. This node uses the disturbance estimator 
- `control_exp` - Control a single hovercraft adding some noise to a standard LQR controller. Used for System Identification.
- `control_signal` - Single motor SysId. Test-stand with a force sensor to measure each motor individually. Removed from CMakeLists to speed-up compile time.

## ToDos
- Set a parameter for the LQR node whether to use the disturbance estimator topic or not.
- Update the control_exp to use the disturbance estimator?