# Air hockey shot planning

A hovercraft attacks the puck: a planner chooses where and how to hit it so it reaches the goal, and a controller makes the hovercraft follow that choice.

## Language

**Shot**:
The robot-puck impact and the puck flight it produces, up to the goal.
_Avoid_: attack, strike

**Bounce sequence**:
The ordered walls the puck hits between the impact and the goal (at most K of them).
_Avoid_: path, route

**Candidate**:
The solved shot for one bounce sequence.
_Avoid_: option, solution

**Plan**:
The candidate the controller follows, from its start time t0 until its impact.
_Avoid_: main plan, trajectory

**Tick**:
One planning call, at the inner rate.
_Avoid_: cycle, iteration

**Inner update**:
A tick that re-solves only the bounce sequence of the plan.

**Outer update**:
A tick that re-solves every bounce sequence and selects the cheapest valid candidate as the plan.
_Avoid_: full re-plan

**Commit**:
Freezing the plan's bounce sequence once its impact is closer than t_commit; later ticks re-solve only that sequence.
