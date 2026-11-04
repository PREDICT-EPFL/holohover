"""Bilevel gradient-based shot MPC: the planner of holohover_planner and the controller of
holohover_control, the code of the ROS nodes shot_planner and control_plan, run in python-sim
(formulation: docs/air-hockey-mpc-formulation.md).

    controller.py  robot 1's closed loop (planner, then controller, at each control step),
                   logging helpers; ShotFactory
    frames.py      python-sim's config as the planner's parameters, world <-> table <-> planner frames
"""
