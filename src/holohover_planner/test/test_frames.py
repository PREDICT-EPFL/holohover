import numpy as np
import pytest

from holohover_planner.shot_mpc.params import Frame, load_parameters

from conftest import CONFIG


@pytest.mark.parametrize("goal", ["left", "right"])
def test_round_trip(P, goal):
    fr = Frame(P, goal)
    p = np.array([[0.3, -0.2], [-0.9, 0.4]])
    assert np.allclose(fr.pos_to_table(fr.pos_to_planner(p)), p)
    assert np.allclose(fr.vec_to_table(fr.vec_to_planner(p)), p)


@pytest.mark.parametrize("goal, sign", [("left", -1), ("right", 1)])
def test_attacked_goal_is_planner_far_end(P, goal, sign):
    """The attacked goal maps to x_p = X, our goal to x_p = 0, the table centre to the midline."""
    fr = Frame(P, goal)
    assert np.allclose(fr.pos_to_planner([sign * 0.5 * P.X, 0.0]), [P.X, 0.0])
    assert np.allclose(fr.pos_to_planner([-sign * 0.5 * P.X, 0.0]), [0.0, 0.0])
    assert np.allclose(fr.pos_to_planner([0.0, 0.0]), [P.x_mid, 0.0])


def test_left_matches_python_sim_world(P):
    """python-sim: world = table + (X/2, W/2), planner = (X - x_w, W/2 - y_w)."""
    fr = Frame(P, "left")
    p_t = np.array([0.4, 0.1])
    x_w, y_w = p_t + [0.5 * P.X, 0.5 * P.W]
    assert np.allclose(fr.pos_to_planner(p_t), [P.X - x_w, 0.5 * P.W - y_w])


def test_bad_goal(P):
    with pytest.raises(ValueError):
        Frame(P, "up")


def test_planner_walls_are_the_wall_faces(P):
    """The planner's table is the Box2D walls' faces, wall_inset inside the table edges, centred on the table."""
    p = load_parameters(CONFIG)
    d = p["table.wall_inset"]
    assert d > 0
    fr = Frame(P, "left")
    face_x, face_y = 0.5 * p["table.width"] - d, 0.5 * p["table.height"] - d
    assert np.allclose(fr.pos_to_planner([-face_x, face_y]), [P.X, -0.5 * P.W])  # attacked end, left side
    assert np.allclose(fr.pos_to_planner([face_x, -face_y]), [0.0, 0.5 * P.W])
