"""Python controllers of a single hovercraft, without ROS (the nodes are thin wrappers):

    plan.py             a planner's plan as the controller follows it, and the ZOH model
    tracker.py          tracking QP on a plan (CasADi + DAQP)
    plan_controller.py  modes (attack a plan, defend at home), plan expiry, defence PD
"""
