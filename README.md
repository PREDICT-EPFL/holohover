# Holohover

This repository contains the ROS2 packages necessary to run the Holohover system.

## Setup

The ROS2 system is designed to run on Docker containers. Setup scripts and documentation can be found in the [Holohover Docker repository](https://github.com/grilloandrea6/holohover-docker/).

## Starting the System

The `holohover_utils` package includes the ROS launch files required to start the system. Refer to the [holohover_utils README](./holohover_utils/README.md) for detailed instructions.

## Packages Overview

Below is a summary of the packages in this repository. For detailed information, consult the README files within each package.

- **`holohover_firmware`**: Firmware for the ESP32 used in the earlier version of the hovercraft.
- **`holohover_common`**: Shared code and utilities for other packages.  
  *ToDo*: Are the `signal_compensation/python_bindings` files still in use?
- **`holohover_control`**: Nodes for controlling a single hovercraft, including control signal generation for System Identification.
- **`holohover_dmpc`**: Implementation of the DMPC distributed controller.
- **`holohover_drivers`**: Code for the Radxa SBCs to control motors and interface with sensors.
- **`holohover_mpc`**: Code for the MPC controller of a single hovercraft.
- **`holohover_msgs`**: Custom ROS2 message definitions.
- **`holohover_navigation`**: Pose estimation using an Extended Kalman Filter (EKF).
- **`holohover_simulator`**: Multibody physics simulator for experiments with multiple hovercraft.
- **`holohover_utils`**: Interface nodes, RViz configurations, launch files, and other utilities.  
  *ToDo*: Consolidate with `holohover_common`?
- **`matlab_experiments_analysis`**: Scripts for analyzing experiments in MATLAB.  
  *ToDo*: Confirm if this package is still required.
- **`micro_ros`**: Submodules for Micro-ROS integration, including [Micro-ROS Agent](https://github.com/micro-ROS/micro-ROS-Agent) and [Micro-ROS Messages](https://github.com/micro-ROS/micro_ros_msgs).
- **`mocap_optitrack`**: Submodule for Optitrack motion capture integration, sourced from [PREDICT-EPFL](https://github.com/PREDICT-EPFL/mocap_optitrack).

> **Note**: The `holohover_firmware` and `micro_ros` packages are relevant only for older hovercraft versions.

## Submodules

This repository includes the following submodules:

- **`LAOPT`**
- **`mocap_optitrack`**
- **`micro_ros`**:
  - `micro_ros_msgs`
  - `micro_ros_agent`
  - `micro_ros_espidf_component`

## Outstanding Tasks (ToDos)

1. Clean `holohover_control/control_lqr_node` of DMPC logging.
2. Merge `holohover_common` and `holohover_utils`?
3. Verify if `holohover_navigation/holohover_ekf.hpp` is utilized. Is it necessary for onboard EKF using IMU and mouse sensors?
4. Is `matlab_experiments_analysis` package useful?
5. Is `holohover_control/control_signal_node` useful?

## Credits

*ToDo*

## License

*ToDo*
