"""The two shells hand the controller the same plan: a PlanSet published by the shot_planner node
(plan_msg) and read by the control_plan node (chosen_plan) gives the plan python-sim hands over
directly (Plan.from_candidate), and both controllers then command the same accelerations.

The messages need ROS (rclpy, holohover_msgs): outside ROS the test is skipped."""

from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("rclpy")
from builtin_interfaces.msg import Time  # noqa: E402

from holohover_control.control_plan_node import chosen_plan  # noqa: E402
from holohover_control.plan import Plan  # noqa: E402
from holohover_control.plan_controller import ATT, DEF, PD, PlanController  # noqa: E402
from holohover_control.tracker import Tracker  # noqa: E402
from holohover_planner.shot_mpc.params import Frame, ShotParams, load_parameters  # noqa: E402
from holohover_planner.shot_mpc.shot_nlp import ShotNLP  # noqa: E402
from holohover_planner.shot_planner import Body, ShotPlanner  # noqa: E402
from holohover_planner.shot_planner_node import plan_msg  # noqa: E402

T0 = 12.35  # s, a tick time that is not a whole number of nanoseconds in binary
PLANNER_CONFIG = Path(__file__).resolve().parents[2] / "holohover_utils/config/common/shot_planner_config.yaml"


@pytest.fixture(scope="module")
def planner():
    P = ShotParams.from_parameters(load_parameters(PLANNER_CONFIG))
    return ShotPlanner(P, Frame(P, "left"), ShotNLP(P, verbose=False))


@pytest.fixture(scope="module")
def plans(planner):
    """The PlanSet of a first tick: h0 at the experiment's start, the puck at rest in front of it."""
    plans = planner.tick(T0, Body(T0, 0.835, 0.32, 0.0, 0.0), Body(T0, 0.385, -0.13, 0.0, 0.0))
    assert plans is not None and plans.chosen >= 0
    return plans


def stamp(t):
    ns = int(round(t * 1e9))
    return Time(sec=ns // 10**9, nanosec=ns % 10**9)


def controller(P):
    acc_max = P.u_max
    tracker = Tracker(P.gamma, acc_max, 10, 0.0, Q=[1e4, 1e4, 1e2, 1e2, 1e2, 1.0],
                      Q_f=[1e6, 1e6, 1e2, 1e4, 1e4, 1.0], R=[1.0, 1.0, 1.0], period=0.01)
    return PlanController(tracker, PD([0.7455, 0.0, 0.0], [43.1, 43.1, 100.0], [8.62, 8.62, 20.0]), acc_max,
                          P.plan_expiry)


def test_published_plan_is_python_sims(plans):
    ros = chosen_plan(plan_msg(stamp(T0), plans))
    sim = Plan.from_candidate(plans.plan)
    assert ros.label == sim.label and ros.tf == sim.tf
    np.testing.assert_array_equal(ros.X, sim.X)  # float64 fields: exact
    np.testing.assert_array_equal(ros.U, sim.U)
    assert abs(ros.t0 - sim.t0) < 1e-9  # builtin_interfaces/Time: whole nanoseconds


def test_every_candidate_is_published(plans):
    msg = plan_msg(stamp(T0), plans)
    assert [c.label for c in msg.candidates] == plans.labels and msg.chosen == plans.chosen
    for c, m in zip(plans.candidates, msg.candidates):
        assert (c is None) == (len(m.states) == 0)
        if c is not None:
            assert m.cost == c.cost and len(m.acc_x) == len(c.acc)


def test_both_controllers_command_the_same(planner, plans):
    ros, sim = controller(planner.P), controller(planner.P)
    ros.on_plan(chosen_plan(plan_msg(stamp(T0), plans)), T0)
    sim.on_plan(Plan.from_candidate(plans.plan), T0)
    x = np.array([0.835, 0.32, 0.05, 0.0, 0.0, 0.0])
    for k in range(10):
        t = T0 + 0.01 * k
        np.testing.assert_allclose(ros.step(t, x), sim.step(t, x), rtol=1e-7, atol=1e-9)
    assert ros.mode == sim.mode == ATT


def test_no_plan_after_a_contact(planner, plans):
    none = planner.on_contact(T0 + 0.2)
    assert none is not None and chosen_plan(plan_msg(stamp(T0 + 0.2), none)) is None
    ctrl = controller(planner.P)
    ctrl.on_plan(Plan.from_candidate(plans.plan), T0)
    ctrl.on_plan(chosen_plan(plan_msg(stamp(T0 + 0.2), none)), T0 + 0.2)
    assert ctrl.mode == DEF
