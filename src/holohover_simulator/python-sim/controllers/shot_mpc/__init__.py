"""Bilevel gradient-based shot MPC (see docs/air-hockey-mpc-formulation.md).

The outer loop enumerates wall-bounce sequences, the inner loop solves one
smooth NLP per sequence with CasADi + IPOPT, and a tracking QP follows the
selected plan. All planning happens in the planner frame (x toward the opponent
goal, wall L at +y); params.py holds the single world <-> planner transform.

    params.py           configuration, world <-> planner frame and unit conversions
    model.py            shared physics: puck flight, ZOH map, impact, drawn puck paths
    shot_nlp.py         inner NLP P(B) for one sequence, and its warm/cold initial guesses
    shot_selector.py    outer loop over the bounce sequences
    tracker.py          tracking QP
    solve_pool.py       worker processes of the outer updates
    shot_controller.py  closed loop, state machine and logging helpers; ShotFactory
    report.py           selector updates as plain data, solve-time summary
"""
