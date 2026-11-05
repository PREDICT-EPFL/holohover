"""
Writes the Foxglove layout of teleop.launch.py for one hovercraft, gui/teleop_<name>.foxglove.json: Foxglove topic
paths cannot take a variable, so there is one layout per name.

    python3 teleop_layout.py h0 h3

Table: the robot (EKF, hN/state_disturbance), the OptiTrack pose (hN/pose), the carrot (hN/state_ref) with its
velocity (= the command), the tracking error. Plots: x, y, yaw, velocities against the carrot; the input (/joy,
hN/cmd_vel); the motor signals (hN/control); the EKF's disturbance estimate.
"""

import json
import os
import sys

SCRIPT = """\
// Teleop scene, frame "world" (table frame, origin at the centre): the table, NAME from its EKF state (blue), its
// OptiTrack pose (white ring), the table frame's axes and corners, the OptiTrack world frame, the carrot NAME/state_ref (orange ghost) with its velocity arrow (= the command)
// and the tracking error (red line, robot to carrot).
import { Input } from "./types";
import { SceneUpdate, SceneEntity, LinePrimitive, CylinderPrimitive, TextPrimitive, Color, Point3 } from "@foxglove/schemas";

type Time = { sec: number; nsec: number };

export const inputs = ["/NAME/state_disturbance", "/NAME/state_ref", "/NAME/pose", "/optitrack/table_pose_raw"];
export const output = "/studio_script/teleop";

const WIDTH = 2.13, HEIGHT = 1.06, GOAL_WIDTH = 0.34, R_ROBOT = 0.07;
// teleop_node.py: the carrot's bounds
const X_MAX = 0.9, Y_MAX = 0.4;
const AXIS = 0.3;  // m, the frame axes' length

const identity = { x: 0, y: 0, z: 0, w: 1 };
const pose = { position: { x: 0, y: 0, z: 0 }, orientation: identity };
const rgba = (r: number, g: number, b: number, a = 1): Color => ({ r, g, b, a });
const pt = (x: number, y: number, z = 0.02): Point3 => ({ x, y, z });

const ROBOT = rgba(0.2, 0.6, 1), CARROT = rgba(1, 0.55, 0.1), WALL = rgba(0.85, 0.85, 0.85);
let robot = { x: 0, y: 0 }, carrot = { x: 0, y: 0 };

function line(points: Point3[], color: Color, thickness = 0.008): LinePrimitive {
  return { type: 0, pose, thickness, scale_invariant: false, points, color, colors: [], indices: [] };
}

function disc(x: number, y: number, radius: number, color: Color, z = 0.01): CylinderPrimitive {
  return { pose: { position: { x, y, z }, orientation: identity },
           size: { x: 2 * radius, y: 2 * radius, z: 0.02 }, bottom_scale: 1, top_scale: 1, color };
}

function circle(x: number, y: number, r: number): Point3[] {
  return Array.from({ length: 33 }, (_, i) => pt(x + r * Math.cos(i * Math.PI / 16), y + r * Math.sin(i * Math.PI / 16), 0.04));
}

function body(x: number, y: number, yaw: number, color: Color): [LinePrimitive, CylinderPrimitive] {
  const heading = [pt(x, y, 0.03), pt(x + R_ROBOT * Math.cos(yaw), y + R_ROBOT * Math.sin(yaw), 0.03)];
  return [line(heading, rgba(1, 1, 1, color.a), 0.01), disc(x, y, R_ROBOT, color)];
}

function text(p: Point3, s: string, color: Color): TextPrimitive {
  return { pose: { position: p, orientation: identity }, billboard: true, font_size: 12, scale_invariant: true,
           color, text: s };
}

function entity(timestamp: Time, id: string, lines: LinePrimitive[], cylinders: CylinderPrimitive[] = [],
                texts: TextPrimitive[] = []): SceneEntity {
  return { timestamp, frame_id: "world", id, lifetime: { sec: 0, nsec: 0 }, frame_locked: false, metadata: [],
           arrows: [], cubes: [], spheres: [], cylinders, lines, triangles: [], texts, models: [] };
}

function table(ts: Time): SceneEntity {
  const x = WIDTH / 2, y = HEIGHT / 2, g = GOAL_WIDTH / 2;
  return entity(ts, "table", [
    line([pt(-x, -y), pt(x, -y), pt(x, y), pt(-x, y), pt(-x, -y)], WALL, 0.01),
    line([pt(0, -y), pt(0, y)], rgba(0.85, 0.85, 0.85, 0.4), 0.004),
    line([pt(-x, -g), pt(-x, g)], rgba(1, 0.2, 0.2), 0.025),
    line([pt(x, -g), pt(x, g)], rgba(1, 0.2, 0.2), 0.025),
    line([pt(-X_MAX, -Y_MAX), pt(X_MAX, -Y_MAX), pt(X_MAX, Y_MAX), pt(-X_MAX, Y_MAX), pt(-X_MAX, -Y_MAX)],
         rgba(1, 0.55, 0.1, 0.35), 0.004),  // the carrot's bounds
    // the table frame: origin at the centre, x (red) to the right goal, y (green) up
    line([pt(0, 0, 0.06), pt(AXIS, 0, 0.06), pt(AXIS - 0.04, 0.025, 0.06), pt(AXIS, 0, 0.06), pt(AXIS - 0.04, -0.025, 0.06)],
         rgba(1, 0.2, 0.2), 0.012),
    line([pt(0, 0, 0.06), pt(0, AXIS, 0.06), pt(0.025, AXIS - 0.04, 0.06), pt(0, AXIS, 0.06), pt(-0.025, AXIS - 0.04, 0.06)],
         rgba(0.2, 1, 0.2), 0.012),
  ], [], [
    text(pt(AXIS + 0.04, 0, 0.06), "x", rgba(1, 0.2, 0.2)),
    text(pt(0, AXIS + 0.04, 0.06), "y", rgba(0.2, 1, 0.2)),
    text(pt(0, -0.04, 0.06), "(0, 0)", WALL),
    ...[[x, y], [-x, y], [-x, -y], [x, -y]].map(([cx, cy]) =>
      text(pt(cx, cy, 0.06), `(${cx.toFixed(3)}, ${cy.toFixed(2)})`, WALL)),
  ]);
}

function error(ts: Time): SceneEntity {
  return entity(ts, "error", [line([pt(robot.x, robot.y, 0.05), pt(carrot.x, carrot.y, 0.05)], rgba(1, 0.2, 0.2), 0.006)]);
}

export default function script(
  event: Input<"/NAME/state_disturbance"> | Input<"/NAME/state_ref"> | Input<"/NAME/pose">
         | Input<"/optitrack/table_pose_raw">,
): SceneUpdate {
  const ts = event.receiveTime;
  if (event.topic === "/optitrack/table_pose_raw") {
    // the OptiTrack world frame in the table frame (optitrack_interface's to_table_frame applied to its origin)
    const p = event.message.pose.position, q = event.message.pose.orientation;
    const yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    const c = Math.cos(yaw), sn = Math.sin(yaw);
    const ox = -c * p.x - sn * p.y, oy = sn * p.x - c * p.y;
    const axis = (a: number, color: Color) =>
      line([pt(ox, oy, 0.06), pt(ox + AXIS * Math.cos(a), oy + AXIS * Math.sin(a), 0.06)], color, 0.012);
    return { deletions: [], entities: [entity(ts, "world", [axis(-yaw, rgba(1, 0.2, 0.2)), axis(Math.PI / 2 - yaw, rgba(0.2, 1, 0.2))],
      [], [text(pt(ox, oy - 0.05, 0.06), "OptiTrack world", WALL),
           text(pt(0, -HEIGHT / 2 - 0.08, 0.06),
                `table in world: (${p.x.toFixed(3)}, ${p.y.toFixed(3)}), yaw ${(yaw * 180 / Math.PI).toFixed(1)}°`, WALL)])] };
  }
  if (event.topic === "/NAME/state_disturbance") {
    const s = event.message.state_msg;
    robot = { x: s.x, y: s.y };
    const [h, d] = body(s.x, s.y, s.yaw, ROBOT);
    return { deletions: [], entities: [table(ts), entity(ts, "robot", [h], [d]), error(ts)] };
  }
  if (event.topic === "/NAME/state_ref") {
    const s = event.message;
    carrot = { x: s.x, y: s.y };
    const [h, d] = body(s.x, s.y, s.yaw, rgba(1, 0.55, 0.1, 0.45));
    const v = line([pt(s.x, s.y, 0.04), pt(s.x + 0.5 * s.v_x, s.y + 0.5 * s.v_y, 0.04)], CARROT, 0.012);  // 0.5 s ahead
    return { deletions: [], entities: [entity(ts, "carrot", [h, v], [d]), error(ts)] };
  }
  const p = event.message.pose.position;
  return { deletions: [], entities: [table(ts), entity(ts, "optitrack", [line(circle(p.x, p.y, R_ROBOT), rgba(1, 1, 1), 0.005)])] };
}
"""


def plot(title, paths):
    return {"foxglovePanelTitle": title,
            "paths": [{"value": v, "enabled": True, "timestampMethod": "receiveTime", "label": label}
                      for label, v in paths],
            "showXAxisLabels": True, "showYAxisLabels": True, "showLegend": True, "legendDisplay": "floating",
            "showPlotValuesInLegend": True, "isSynced": True, "xAxisVal": "timestamp", "sidebarDimension": 240}


def layout(name):
    s, r = f"/{name}/state_disturbance.state_msg", f"/{name}/state_ref"
    motors = [f"motor_{m}_{i}" for m in "abc" for i in (1, 2)]
    config = {
        "3D!teleop": {
            "foxglovePanelTitle": f"Table: {name} (blue), OptiTrack (ring), carrot (orange, arrow = command), error (red)",
            "cameraState": {"perspective": False, "distance": 2.6, "phi": 0, "thetaOffset": 0, "targetOffset": [0, 0, 0],
                            "target": [0, 0, 0], "targetOrientation": [0, 0, 0, 1], "fovy": 45, "near": 0.01, "far": 100},
            "followMode": "follow-none", "followTf": "world",
            "scene": {"transforms": {"showLabel": False, "axisScale": 0.5}, "meshUpAxis": "z_up"},
            "transforms": {"frame:world": {"visible": False}},
            "topics": {"/studio_script/teleop": {"visible": True}}, "layers": {}},
        "Plot!x": plot("x [m]", [("EKF", f"{s}.x"), ("carrot", f"{r}.x"), ("OptiTrack", f"/{name}/pose.pose.position.x")]),
        "Plot!y": plot("y [m]", [("EKF", f"{s}.y"), ("carrot", f"{r}.y"), ("OptiTrack", f"/{name}/pose.pose.position.y")]),
        "Plot!yaw": plot("yaw [rad]", [("EKF", f"{s}.yaw"), ("carrot", f"{r}.yaw")]),
        "Plot!v": plot("velocity [m/s]", [("vx", f"{s}.v_x"), ("vx carrot", f"{r}.v_x"),
                                          ("vy", f"{s}.v_y"), ("vy carrot", f"{r}.v_y")]),
        "Plot!w": plot("yaw rate [rad/s]", [("w_z", f"{s}.w_z"), ("w_z carrot", f"{r}.w_z")]),
        "Plot!cmd": plot(f"Command /{name}/cmd_vel", [("vx", f"/{name}/cmd_vel.linear.x"), ("vy", f"/{name}/cmd_vel.linear.y"),
                                                      ("w_z", f"/{name}/cmd_vel.angular.z")]),
        "Plot!joy": plot("Gamepad /joy (L1 enable, R1 turbo)", [("left x (axis 0)", "/joy.axes[0]"), ("left y (axis 1)", "/joy.axes[1]"),
                                                                ("right x (axis 2)", "/joy.axes[2]"), ("L1", "/joy.buttons[9]"),
                                                                ("R1", "/joy.buttons[10]")]),
        "Plot!motors": plot(f"Motor signals /{name}/control", [(m, f"/{name}/control.{m}") for m in motors]),
        "Plot!dist": plot("EKF disturbance", [("x", f"{s}.dist_x"), ("y", f"{s}.dist_y"), ("yaw", f"{s}.dist_yaw")]),
        "RosOut!log": {"foxglovePanelTitle": "Log", "searchTerms": [], "minLogLevel": 2, "nameFilter": {}},
    }
    row = lambda a, b, p=50: {"direction": "row", "splitPercentage": p, "first": a, "second": b}
    col = lambda a, b, p=50: {"direction": "column", "splitPercentage": p, "first": a, "second": b}
    plots = col(col(row("Plot!x", "Plot!y"), row("Plot!yaw", "Plot!w")),
                col(row("Plot!v", "Plot!cmd"), row("Plot!joy", row("Plot!motors", "Plot!dist"), 40)), 50)
    config["Tab!main"] = {"activeTabIdx": 0, "tabs": [{"title": f"Teleop {name}",
                                                       "layout": row(col("3D!teleop", "RosOut!log", 75), plots, 40)}]}
    return {"configById": config, "globalVariables": {},
            "userNodes": {"teleop-scene": {"name": "teleop scene", "sourceCode": SCRIPT.replace("NAME", name)}},
            "playbackConfig": {"speed": 1}, "layout": "Tab!main"}


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    for name in sys.argv[1:]:
        path = os.path.join(here, f"teleop_{name}.foxglove.json")
        with open(path, "w") as f:
            json.dump(layout(name), f, indent=2)
        print(path)
