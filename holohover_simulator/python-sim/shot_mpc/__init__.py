"""Bilevel gradient-based shot MPC (see air-hockey-mpc-formulation.typ).

The outer loop enumerates wall-bounce sequences, the inner loop solves one
smooth NLP per sequence with CasADi + IPOPT. All planning happens in the
planner frame (x toward the opponent goal, wall L at +y); params.py holds the
single world <-> planner transform.
"""
