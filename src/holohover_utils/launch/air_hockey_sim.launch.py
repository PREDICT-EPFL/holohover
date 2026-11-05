"""
Air hockey in the Box2D simulation: the simulator with the air-hockey table and puck
(config/common/simulation_air_hockey.yaml), the optitrack interface, the puck's EKF and, per hovercraft,
its EKF (navigation_disturbance), the shot MPC planner (holohover_planner shot_planner), the plan
controller (holohover_control control_plan, which sends the motor signals).

    ros2 launch holohover_utils air_hockey_sim.launch.py
    ros2 launch holohover_utils air_hockey_sim.launch.py puck:="0.385,-0.13"
    ros2 launch holohover_utils air_hockey_sim.launch.py evaluate:=50 first:=0

evaluate: run that many of python-sim's stored starts from start `first` (air_hockey_evaluate.py, results
in /root/data/box2d_eval), then stop the launch; the planner and the controller wait for its episodes.
python_sim: python-sim's directory, where the evaluation reads its starts and settings (default: the one
next to this file's source, found through the --symlink-install link; set it for any other install).

puck: x, y of the puck at rest, table frame (default: the experiment file's puck).
substeps: Box2D steps per simulator period (default 10, 1 ms steps). Box2D resolves a contact one of its steps
after the overlap appears.
controller: plan (default) or teleop: per hovercraft control_lqr and holohover_control teleop instead of the
planner and control_plan, driven by a Twist on hN/cmd_vel. teleop.launch.py starts it with the gamepad or
keyboard nodes.
record: record every topic to an MCAP bag, /root/data/rosbags/air_hockey_<date>_<time> (docker/data on the
host), stopped with the launch. It opens in Foxglove with the same layout.

Foxglove (ws://localhost:8765, or a recorded bag): import the layout gui/air_hockey.foxglove.json, which
python-sim's MCAP files use too. Its user script draws the table, h0, the puck, h0's plan and the
predicted shot from /h0/state, /puck/state and /h0/plan (/studio_script/air_hockey). Its second tab is for
python-sim's own topics.
"""

import os
import sys
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

this_dir = os.path.dirname(os.path.abspath(__file__))
if this_dir not in sys.path:
    sys.path.insert(0, this_dir)
# python-sim next to this file's source (src/holohover_utils/launch -> src/holohover_simulator/python-sim)
PYTHON_SIM = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', 'holohover_simulator', 'python-sim')

import experiment_config_parser as ecp_lib


def launch_setup(context):
    experiment_filename = LaunchConfiguration('experiment').perform(context)
    puck_arg = LaunchConfiguration('puck').perform(context)
    record = LaunchConfiguration('record').perform(context).lower() == 'true'
    evaluate = int(LaunchConfiguration('evaluate').perform(context) or 0)
    first = int(LaunchConfiguration('first').perform(context) or 0)
    python_sim = os.path.normpath(LaunchConfiguration('python_sim').perform(context) or PYTHON_SIM)
    substeps = int(LaunchConfiguration('substeps').perform(context) or 10)
    teleop = LaunchConfiguration('controller').perform(context) == 'teleop'

    utils_share = get_package_share_directory('holohover_utils')
    common = os.path.join(utils_share, 'config', 'common')
    ecp = ecp_lib.ExperimentConfigParser(experiment_filename)
    sim_config = ecp.getSimConfig()
    with open(ecp.experiment_conf) as f:
        puck = yaml.safe_load(f)['puck']['initial_state']
    if puck_arg:
        puck = [float(v) for v in puck_arg.split(',')]
    print(f"Air hockey experiment {experiment_filename}, puck (x, y) = {puck}")

    simulation_files = [os.path.join(common, name) for name in ('simulation_config.yaml', 'simulation_air_hockey.yaml')]
    navigation_config = os.path.join(common, 'navigation_config.yaml')

    ld = [
        Node(
            package='holohover_simulator',
            executable='simulator',
            parameters=simulation_files + [{
                **{k: sim_config[k] for k in ('hovercraft_ids', 'hovercraft_names', 'initial_state_x',
                                              'initial_state_y', 'initial_state_theta', 'initial_state_vx',
                                              'initial_state_vy', 'initial_state_w', 'holohover_props_files',
                                              'simulated')},
                'puck_position': [float(puck[0]), float(puck[1])],
                'substeps': substeps,
            }],
            output='screen',
        ),
        Node(
            package='holohover_utils',
            executable='optitrack_interface',
            parameters=simulation_files + [sim_config],
            output='screen',
        ),
        Node(
            package='holohover_navigation',
            executable='navigation',
            namespace='puck',
            parameters=[navigation_config],
            remappings=[('pose', '/puck/pose')],
            ros_arguments=['--disable-rosout-logs'],
            output='screen',
        ),
    ]

    if record:
        bag = os.path.join('/root/data/rosbags', f"air_hockey_{datetime.now():%Y%m%d_%H%M%S}")
        os.makedirs(os.path.dirname(bag), exist_ok=True)
        print(f"Recording every topic to {bag}")
        ld.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-s', 'mcap', '-o', bag],
            output='screen',
        ))

    _, names, props_files, initial = ecp.getHovercraft()
    for i, (name, props) in enumerate(zip(names, props_files)):
        ld.append(Node(
            package='holohover_navigation',
            executable='navigation_disturbance',
            namespace=name,
            parameters=[navigation_config, {'holohover_props_file': props}],
            ros_arguments=['--disable-rosout-logs'],
            output='screen',
        ))
        if teleop:
            ld += [
                Node(
                    package='holohover_control',
                    executable='control_lqr',
                    name='lqr',
                    namespace=name,
                    parameters=[os.path.join(common, 'control_lqr_config.yaml'),
                                {'holohover_props_file': props, 'initial_x': initial['x'][i],
                                 'initial_y': initial['y'][i], 'initial_yaw': initial['theta'][i]}],
                    output='screen',
                ),
                Node(
                    package='holohover_control',
                    executable='teleop',
                    namespace=name,
                    output='screen',
                    emulate_tty=True,
                    additional_env={'PYTHONUNBUFFERED': '1'},
                ),
            ]
            continue
        ld += [
            Node(
                package='holohover_control',
                executable='control_plan',
                namespace=name,
                parameters=[os.path.join(common, 'control_plan_config.yaml'),
                            {'holohover_props_file': props, 'start_idle': evaluate > 0}],
                output='screen',
                emulate_tty=True,
                additional_env={'PYTHONUNBUFFERED': '1'},
            ),
            Node(
                package='holohover_planner',
                executable='shot_planner',
                namespace=name,
                parameters=[os.path.join(common, 'shot_planner_config.yaml'), {'start_idle': evaluate > 0}],
                output='screen',
                emulate_tty=True,
                additional_env={'PYTHONUNBUFFERED': '1'},
            ),
        ]
    if evaluate > 0:
        evaluator = Node(
            package='holohover_utils',
            executable='air_hockey_evaluate.py',
            arguments=['--episodes', str(evaluate), '--first', str(first), '--name', names[0],
                       '--python-sim', python_sim],
            output='screen',
            emulate_tty=True,
            additional_env={'PYTHONUNBUFFERED': '1'},
        )
        ld += [evaluator, RegisterEventHandler(OnProcessExit(target_action=evaluator,
                                                             on_exit=[EmitEvent(event=Shutdown())]))]
    return ld


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('experiment', default_value='air_hockey.yaml', description='Experiment file'),
        DeclareLaunchArgument('puck', default_value='', description='Puck x,y at rest (table frame); a moving start goes through the simulator ~/reset'),
        DeclareLaunchArgument('record', default_value='false', description='Record every topic to an MCAP bag'),
        DeclareLaunchArgument('evaluate', default_value='0', description="Evaluate on that many of python-sim's starts"),
        DeclareLaunchArgument('first', default_value='0', description='First start of the evaluation'),
        DeclareLaunchArgument('substeps', default_value='10', description='Box2D steps per simulator period'),
        DeclareLaunchArgument('controller', default_value='plan', description='plan (shot planner) or teleop (control_lqr + teleop, cmd_vel)'),
        DeclareLaunchArgument('python_sim', default_value='', description="python-sim's directory (default: next to this file's source)"),
        OpaqueFunction(function=launch_setup),
    ])
