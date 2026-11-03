"""Bilevel gradient-based shot MPC planner (see python-sim docs/air-hockey-mpc-formulation.md).

The outer loop enumerates wall-bounce sequences, the inner loop solves one smooth NLP per
sequence with CasADi + Fatrop. All planning happens in the planner frame (x toward the
opponent goal, wall L at +y); params.Frame maps it from and to the table frame.

    params.py         configuration (ROS parameters), table <-> planner frame
    model.py          shared physics: puck flight, ZOH map, impact, drawn puck paths
    shot_nlp.py       inner NLP P(B) for one sequence, and its warm/cold initial guesses
    shot_selector.py  outer loop over the bounce sequences
    solve_pool.py     worker processes of the outer updates
    report.py         selector updates as plain data, solve-time summary

The tracker and the closed loop belong to the controller (holohover_control).
"""
