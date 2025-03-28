# Holohover


[![DOI](https://img.shields.io/badge/DOI-10.48550/arXiv.2409.13334-green.svg)](https://doi.org/10.48550/arXiv.2409.13334) [![Preprint](https://img.shields.io/badge/Preprint-arXiv-blue.svg)](https://arxiv.org/abs/2409.13334) [![Funding](https://img.shields.io/badge/Grant-NCCR%20Automation%20(51NF40\_225155)-90e3dc.svg)](https://nccr-automation.ch/)

This repository contains the ROS2 packages necessary to run the Holohover system.

## Setup

The ROS2 system is designed to run on Docker containers. Setup scripts and documentation can be found in the [Holohover Docker repository](https://github.com/PREDICT-EPFL/holohover-docker).

## Packages Overview

Below is a summary of the packages in this repository. For detailed information, consult the README files within each package.

- **`holohover_firmware`**: Firmware for the ESP32 used in the earlier version of the hovercraft.
- **`holohover_common`**: Shared code and utilities for other packages.  
- **`holohover_control`**: Nodes for controlling a single hovercraft, including control signal generation for System Identification.
- **`holohover_dmpc`**: Implementation of the DMPC distributed controller.
- **`holohover_drivers`**: Code for the Radxa SBCs to control motors and interface with sensors.
- **`holohover_mpc`**: Code for the MPC controller of a single hovercraft.
- **`holohover_msgs`**: Custom ROS2 message definitions.
- **`holohover_navigation`**: Pose estimation using an Extended Kalman Filter (EKF).
- **`holohover_simulator`**: Multibody physics simulator for experiments with multiple hovercraft.
- **`holohover_utils`**: Interface nodes, RViz configurations, launch files, and other utilities.  
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

## Citing our Work

To cite our work in other academic papers, please use the following BibTex entry:
```
@INPROCEEDINGS{stomberg2025,
  author={Stomberg, Gösta and Schwan, Roland and Grillo, Andrea and Jones, Colin N. and Faulwasser, Timm},
  booktitle={International Conference on Robotics and Automation (ICRA)}, 
  title={Cooperative distributed model predictive control for embedded systems: Experiments with hovercraft formations}, 
  year={2025}}
```

## License

BSD 2-Clause
