"""
The shot MPC planner without ROS: the ROS node (shot_planner_node.py) and python-sim
(holohover_simulator/python-sim) both run this class. States and plans are in the table frame,
times in s.

    planner = ShotPlanner(P, Frame(P, "left"), nlp)
    plans = planner.tick(t, robot, puck)     # at shot_mpc.rates.inner_hz: a PlanSet, or None
    plans = planner.on_contact(t)            # a robot-puck contact: the PlanSet with no plan, or None
    planner.reset()                          # a new episode: start afresh at the next tick

Each tick re-solves the bounce sequence of the plan being followed (inner update); every
inner_hz / outer_hz ticks all sequences are re-solved and the cheapest valid one is chosen (outer
update). Without a plan, only the outer updates run. Closer than t_commit to the impact the
sequence is committed, closer than t_stop_replan the plan is no longer re-solved, and a plan whose
impact passed by more than plan_expiry is dropped.

A plan starts at the tick's time t (its candidates' t0), from the robot and puck states advanced
from their stamps to t with the planner's models: the robot under the last input, the puck in free
flight with its wall reflections.

A contact drops the plan and the warm starts at the next tick and pauses the planning for
contact_cooldown. on_contact and reset may run in another thread than tick (the node's callbacks):
they only set flags, and a tick whose solve overlaps a contact discards its result.
"""

from dataclasses import dataclass

import numpy as np

from .shot_mpc.model import discrete_matrices, flight_after, predict
from .shot_mpc.shot_selector import ShotSelector, label


@dataclass
class Body:
    """A body's measured state at time t, table frame: position (x, y) and velocity (vx, vy)."""
    t: float
    x: float
    y: float
    vx: float
    vy: float


@dataclass
class Candidate:
    """One bounce sequence's plan, table frame: the states (x, y, vx, vy) at the N + 1 nodes
    t0 + k tf / N up to the impact, and the acceleration (a_x, a_y) held on each of the N intervals."""
    label: str
    cost: float
    t0: float
    tf: float
    states: np.ndarray  # (N + 1, 4)
    acc: np.ndarray  # (N, 2)
    impact_puck: np.ndarray  # (2,) puck position at the impact
    puck_velocity: np.ndarray  # (2,) puck velocity right after the impact


@dataclass
class PlanSet:
    """What the planner publishes after an update: one entry per bounce sequence, in the order of
    labels (None: no valid plan), and the index of the plan to follow (-1: none)."""
    labels: list
    candidates: list
    chosen: int = -1

    @property
    def plan(self):
        """The candidate to follow, or None."""
        return self.candidates[self.chosen] if self.chosen >= 0 else None


def candidates(selector, plan):
    """[(label, shot_selector.Plan or None)], one per sequence of the selector in its order: the plan
    being followed for its sequence, otherwise the sequence's latest solve if valid, otherwise None. A
    sequence not re-solved at an inner update keeps its result (and its t0) from the last outer update."""
    out = []
    for s, k in selector.seqs:
        if plan is not None and plan.key == (s, k):
            entry = plan
        else:
            entry = selector.candidates.get((s, k))
            if entry is not None and not entry.sol.valid:
                entry = None
        out.append((label(s, k), entry))
    return out


class ShotPlanner:
    def __init__(self, P, frame, nlp, max_state_age=np.inf, log=None):
        """P: ShotParams. frame: params.Frame (the attacked goal). nlp: ShotNLP, shared by planners
        (python-sim builds one per process). max_state_age: s, no planning from older states.
        log: callable(str) for the plan switches and drops, None for silence."""
        self.P, self.frame = P, frame
        self.selector = ShotSelector(P, nlp)
        self.labels = [label(*seq) for seq in self.selector.seqs]
        self.outer_every = max(1, int(round(P.inner_hz / P.outer_hz)))
        self.max_state_age = max_state_age
        self.log = log or (lambda message: None)

        self.plan = None  # the plan to follow (shot_selector.Plan), planner frame
        self.adopted = None  # the plan adopted by the last tick (a new solve), else None
        self.ticks = 0
        self.attack_after = -np.inf  # end of the contact cooldown
        self.contact_pending = False  # a contact the next tick has not handled yet
        self.reset_pending = False  # a reset the next tick has not handled yet

    def none(self):
        """The PlanSet with no plan and no candidates."""
        return PlanSet(self.labels, [None] * len(self.labels))

    def on_contact(self, t):
        """A robot-puck contact at time t: the PlanSet to publish (no plan), or None for a contact
        during the cooldown of an earlier one (ignored)."""
        if t < self.attack_after:
            return None
        self.attack_after = t + self.P.contact_cooldown
        self.contact_pending = True
        return self.none()

    def reset(self):
        """Start afresh at the next tick (a new episode): no plan, no warm starts, no cooldown, and that
        tick an outer update."""
        self.reset_pending = True

    def tick(self, t, robot, puck, u_prev=None):
        """One planning tick at time t from the latest robot and puck states (Body, or None if none
        yet). u_prev: the last applied acceleration (a_x, a_y), table frame; None: the plan's own input
        at t (zero without a plan). Returns the PlanSet to publish, or None if nothing changed."""
        P = self.P
        if self.reset_pending:
            self.reset_pending = self.contact_pending = False
            self.reset_plan()
            self.attack_after = -np.inf
            self.ticks = 0
        outer_tick = self.ticks % self.outer_every == 0
        self.ticks += 1
        self.adopted = None
        out = None

        if self.contact_pending:
            self.contact_pending = False
            self.reset_plan()
        if self.plan is not None and self.plan.remaining(t) < -P.plan_expiry:
            self.log(f"t={t:.2f}s plan {self.plan.label} expired without contact")
            self.reset_plan()
            out = self.none()

        fresh = (robot is not None and puck is not None
                 and max(t - robot.t, t - puck.t) <= self.max_state_age)
        if fresh and t >= self.attack_after and (self.plan is not None or outer_tick):
            out = self.update(t, robot, puck, u_prev, outer_tick) or out
        return out

    def update(self, t, robot, puck, u_prev, outer_tick):
        P, fr = self.P, self.frame
        remaining = self.plan.remaining(t) if self.plan is not None else np.inf
        if remaining <= P.t_stop_replan:
            return None
        if u_prev is None:
            u_prev = self.plan.input_at(t) if self.plan is not None else np.zeros(2)
        else:
            u_prev = fr.vec_to_planner(u_prev)
        # The states at t, planner frame: the robot under the last input, the puck in free flight
        xR = np.concatenate([fr.pos_to_planner([robot.x, robot.y]), fr.vec_to_planner([robot.vx, robot.vy])])
        A_d, B_d = discrete_matrices(max(t - robot.t, 0.0), P.gamma)
        xR = A_d @ xR + B_d @ u_prev
        q, v = flight_after(fr.pos_to_planner([puck.x, puck.y]), fr.vec_to_planner([puck.vx, puck.vy]), P,
                            max(t - puck.t, 0.0))

        committed = self.plan is not None and remaining < P.t_commit
        new = self.selector.update(t, xR, u_prev, predict(q, v, P), outer=outer_tick or self.plan is None,
                                   commit_key=self.plan.key if committed else None)
        if self.contact_pending:  # a contact during the solve: the plans are stale
            return None
        if new is not None:
            if self.plan is None or new.key != self.plan.key:
                sol = new.sol
                self.log(f"t={t:.2f}s plan {new.label}: tf={new.tf:.3f}s cost={sol.cost:.3f} "
                         f"({'cold' if sol.cold else 'warm'}, {sol.iters} it, {1e3 * new.t_spent:.1f} ms)")
            self.plan = self.adopted = new
        return self.plan_set()

    def reset_plan(self):
        self.plan = None
        self.selector.reset()

    def plan_set(self):
        """Every sequence's candidate, and the plan to follow."""
        entries = candidates(self.selector, self.plan)
        chosen = next((i for i, (_, plan) in enumerate(entries) if plan is self.plan), -1) if self.plan else -1
        return PlanSet(self.labels, [None if plan is None else self.candidate(name, plan) for name, plan in entries],
                       chosen)

    def candidate(self, name, plan):
        """A shot_selector.Plan as a Candidate, table frame."""
        sol, fr = plan.sol, self.frame
        return Candidate(
            label=name, cost=float(plan.sol.cost), t0=float(plan.t0), tf=float(plan.tf),
            states=np.hstack([fr.pos_to_table(sol.X[0:2].T), fr.vec_to_table(sol.X[2:4].T)]),
            acc=fr.vec_to_table(sol.U.T),
            impact_puck=fr.pos_to_table(sol.p_c), puck_velocity=fr.vec_to_table(sol.vp),
        )
