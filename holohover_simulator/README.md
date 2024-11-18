# Holohover Simulator

This repository contains the node for simulating the Holohover system, allowing for rapid development, testing, and validation of algorithms in a controlled virtual environment.

## Overview
The simulator leverages the **Box2D physics engine** to model the dynamics of the hovercraft and their interactions between each other with the environment. It simulates impacts between the hovercraft or with the table. It also simulates air drag and forces induced by table tilt.

## Nodes
- `holohover_simulator` - the Simulator node.
  
  The node subscribes to the `control` topic of each simulated hovercraft, and publishes its `/optitrack/*_pose_raw`.

  If the experiment includes both simulated and real hovercraft, the simulator subscribes to the `/optitrack/table_pose_raw` topic.

  Otherwise if the experiment includes only simulated hovercraft, the simulator publishes to the `/optitrack/table_pose_raw` a fixed dummy position of the table that is defined in the config file.

