"""Export python-sim telemetry using the ROS topic and message shapes."""

import json
import time
from datetime import datetime
from pathlib import Path

import numpy as np
from mcap.writer import Writer


POSE_DEFINITIONS = {
    "header": {
        "type": "object",
        "properties": {
            "stamp": {"$ref": "#/definitions/time"},
            "frame_id": {"type": "string"},
        },
    },
    "time": {
        "type": "object",
        "properties": {"sec": {"type": "integer"}, "nanosec": {"type": "integer"}},
    },
    "vector3": {
        "type": "object",
        "properties": {
            "x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"}
        },
    },
    "quaternion": {
        "type": "object",
        "properties": {
            "x": {"type": "number"}, "y": {"type": "number"},
            "z": {"type": "number"}, "w": {"type": "number"}
        },
    },
}

POSE_SCHEMA = {
    "type": "object",
    "properties": {
        "header": {"$ref": "#/definitions/header"},
        "pose": {
            "type": "object",
            "properties": {
                "position": {"$ref": "#/definitions/vector3"},
                "orientation": {"$ref": "#/definitions/quaternion"},
            },
        },
    },
    "definitions": POSE_DEFINITIONS,
}

STATE_SCHEMA = {
    "type": "object",
    "properties": {
        "header": {"$ref": "#/definitions/header"},
        "state_msg": {
            "type": "object",
            "properties": {
                "x": {"type": "number"}, "y": {"type": "number"},
                "v_x": {"type": "number"}, "v_y": {"type": "number"},
                "yaw": {"type": "number"}, "w_z": {"type": "number"},
            },
        },
    },
    "definitions": POSE_DEFINITIONS,
}

CONTROL_SCHEMA = {
    "type": "object",
    "properties": {
        "header": {"$ref": "#/definitions/header"},
        "motor_a_1": {"type": "number"}, "motor_a_2": {"type": "number"},
        "motor_b_1": {"type": "number"}, "motor_b_2": {"type": "number"},
        "motor_c_1": {"type": "number"}, "motor_c_2": {"type": "number"},
    },
    "definitions": {"header": POSE_DEFINITIONS["header"], "time": POSE_DEFINITIONS["time"]},
}

MARKER_SCHEMA = {
    "type": "object",
    "properties": {"markers": {"type": "array", "items": {"type": "object"}}},
}


def _header(stamp_ns, frame_id):
    return {
        "stamp": {"sec": stamp_ns // 1_000_000_000, "nanosec": stamp_ns % 1_000_000_000},
        "frame_id": frame_id,
    }


def _pose(state, stamp_ns):
    x, y, yaw = np.asarray(state)[[0, 1, 4]].tolist()
    half_yaw = yaw / 2.0
    return {
        "header": _header(stamp_ns, "world"),
        "pose": {
            "position": {"x": x, "y": y, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": float(np.sin(half_yaw)), "w": float(np.cos(half_yaw))},
        },
    }


def _state(state, stamp_ns):
    x, y, v_x, v_y, yaw, w_z = np.asarray(state).tolist()
    return {
        "header": _header(stamp_ns, "world"),
        "state_msg": {"x": x, "y": y, "v_x": v_x, "v_y": v_y, "yaw": yaw, "w_z": w_z},
    }


def _control(values, stamp_ns):
    values = np.asarray(values).tolist()
    return {
        "header": _header(stamp_ns, "body"),
        "motor_a_1": values[0], "motor_a_2": values[1], "motor_b_1": values[2],
        "motor_b_2": 0.0, "motor_c_1": 0.0, "motor_c_2": 0.0,
    }


def _marker(stamp_ns, marker_id, marker_type, position, scale, color, namespace):
    return {
        "header": _header(stamp_ns, "world"),
        "ns": namespace,
        "id": marker_id,
        "type": marker_type,
        "action": 0,
        "pose": {
            "position": {"x": position[0], "y": position[1], "z": position[2]},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "scale": {"x": scale[0], "y": scale[1], "z": scale[2]},
        "color": {"r": color[0], "g": color[1], "b": color[2], "a": color[3]},
        "lifetime": {"sec": 0, "nanosec": 0},
        "frame_locked": False,
    }


def _visualization_markers(states, stamp_ns, config, hovercraft_names):
    table = config["table"]
    robot_radius = config["robot"]["radius"]
    puck_radius = config["puck"]["radius"]
    markers = [
        _marker(
            stamp_ns, 0, 1,
            [table["width"] / 2.0, table["height"] / 2.0, -0.01],
            [table["width"], table["height"], 0.02],
            [0.35, 0.35, 0.35, 0.35], "table",
        ),
        _marker(
            stamp_ns, 1, 3,
            [float(states[12]), float(states[13]), 0.01],
            [2.0 * puck_radius, 2.0 * puck_radius, 0.02],
            [1.0, 0.75, 0.05, 1.0], "puck",
        ),
    ]
    colors = ([0.1, 0.35, 1.0, 0.9], [1.0, 0.2, 0.2, 0.9])
    for index, name in enumerate(hovercraft_names):
        state = states[index * 6:(index + 1) * 6]
        markers.append(
            _marker(
                stamp_ns, index + 2, 3,
                [float(state[0]), float(state[1]), 0.02],
                [2.0 * robot_radius, 2.0 * robot_radius, 0.04],
                colors[index % len(colors)], name,
            )
        )
    return {"markers": markers}

def write_mcap(simulation_data, output_path, dt=None, hovercraft_names=None):
    """Write telemetry on the same topics used by the ROS simulator."""
    config = simulation_data["config"]
    if dt is None:
        dt = 1.0 / config["simulation"]["hz"]
    table_period = config["simulation"].get("table_publish_period", 1.0)
    if hovercraft_names is None:
        hovercraft_names = config.get("ros", {}).get("hovercraft_names", ["h0", "h1"])

    states = np.asarray(simulation_data["states"])
    controls = np.asarray(simulation_data["input_sequence"])
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    start_time_ns = time.time_ns()

    with output_path.open("wb") as stream:
        writer = Writer(stream)
        writer.start()
        schemas = {
            "pose": writer.register_schema("geometry_msgs/msg/PoseStamped", "jsonschema", json.dumps(POSE_SCHEMA).encode()),
            "state": writer.register_schema("holohover_msgs/msg/HolohoverStateStamped", "jsonschema", json.dumps(STATE_SCHEMA).encode()),
            "control": writer.register_schema("holohover_msgs/msg/HolohoverControlStamped", "jsonschema", json.dumps(CONTROL_SCHEMA).encode()),
            "markers": writer.register_schema("visualization_msgs/msg/MarkerArray", "jsonschema", json.dumps(MARKER_SCHEMA).encode()),
        }
        channels = {}
        for name in hovercraft_names:
            channels[f"{name}_pose"] = writer.register_channel(f"/optitrack/{name}_pose_raw", "json", schemas["pose"])
            channels[f"{name}_state"] = writer.register_channel(f"/{name}/state", "json", schemas["state"])
            channels[f"{name}_control"] = writer.register_channel(f"/{name}/control", "json", schemas["control"])
        channels["puck_pose"] = writer.register_channel("/optitrack/puck_pose_raw", "json", schemas["pose"])
        channels["table_pose"] = writer.register_channel("/optitrack/table_pose_raw", "json", schemas["pose"])
        channels["markers"] = writer.register_channel("/visualization_marker_array", "json", schemas["markers"])

        for step in range(states.shape[0]):
            stamp_ns = start_time_ns + int(round(step * dt * 1e9))
            for index, name in enumerate(hovercraft_names):
                state = states[step, index * 6:(index + 1) * 6]
                messages = (
                    (f"{name}_pose", _pose(state, stamp_ns)),
                    (f"{name}_state", _state(state, stamp_ns)),
                    (f"{name}_control", _control(controls[step, :3], stamp_ns)),
                )
                for channel_name, message in messages:
                    writer.add_message(channels[channel_name], stamp_ns, json.dumps(message).encode(), stamp_ns)

            puck_state = states[step, 12:18]
            writer.add_message(channels["puck_pose"], stamp_ns, json.dumps(_pose(puck_state, stamp_ns)).encode(), stamp_ns)
            marker_message = _visualization_markers(states[step], stamp_ns, config, hovercraft_names)
            writer.add_message(channels["markers"], stamp_ns, json.dumps(marker_message).encode(), stamp_ns)
            if step == 0 or np.isclose((step * dt) % table_period, 0.0, atol=dt / 2):
                writer.add_message(channels["table_pose"], stamp_ns, json.dumps(_pose([0, 0, 0, 0, 0, 0], stamp_ns)).encode(), stamp_ns)
        writer.finish()
    return output_path


def default_output_path(directory="simulation_data"):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(directory) / f"{timestamp}.mcap"