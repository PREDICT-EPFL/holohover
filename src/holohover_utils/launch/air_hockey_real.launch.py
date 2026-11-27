"""
Air hockey on the real table: the optitrack interface (OptiTrack poses to the table frame), the puck's EKF and,
per hovercraft of the experiment, its EKF (navigation_disturbance), the shot MPC planner (holohover_planner
shot_planner) and the plan controller (holohover_control control_plan, which sends the motor signals). No
simulator: the mocap node (./holohover mocap start) and the hovercraft's own launchFC_loc.sh run separately.

    ros2 launch holohover_utils air_hockey_real.launch.py
    ros2 launch holohover_utils air_hockey_real.launch.py start_idle:=true record:=true

start_idle: the planner and the controller start idle (no plan, no acceleration). Start them with
    ros2 topic pub --once /h3/episode std_msgs/msg/Int32 "{data: 0}"
and idle them again with data: -1. Stopping the launch stops the motors (holohover_fc's watchdog).
record: record every topic to an MCAP bag, /root/data/<name>/<run>_<date>_<time> (docker/data on the host;
name:= defaults to the experiment's name, run counts up per folder), stopped with the launch.

Per hovercraft, the experiment file picks the controller (default: control_plan + shot planner):
    controller: {type: lqr, x: 0.0, y: 0.0, yaw: 0.0}             # hold a pose (hovercraft_lqr.launch.py)
    planner: {active: false}                                     # control_plan only (defence hold), no shot planner
Collision identification (collision_id_real.yaml), moving the LQR's reference live:
    ros2 launch holohover_utils air_hockey_real.launch.py experiment:=collision_id_real.yaml record:=true
    ros2 topic pub --once /h3/state_ref holohover_msgs/msg/HolohoverState "{x: 0.2, y: 0.0, yaw: 0.0}"

Foxglove: the layout gui/air_hockey.foxglove.json draws /h0/...; rename its topics for another hovercraft.
"""

import os
import sys
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

this_dir = os.path.dirname(os.path.abspath(__file__))
if this_dir not in sys.path:
    sys.path.insert(0, this_dir)

import experiment_config_parser as ecp_lib


# The puck's EKF trusts OptiTrack less than navigation_config.yaml's 1e-6: the puck's velocity noise at rest falls
# from 1.7 cm/s (std, 9 cm/s max) to 0.34 cm/s (1.6 cm/s max), the noise that made the plans jitter, for about one
# more OptiTrack sample (4 ms) before a contact is detected and 30 ms instead of 12 ms to settle on the speed after
# a hit. Offline replay of the EKF on air_hockey_real_20261124_164813 (puck at rest) and _133822 (hits).
PUCK_EKF = {'sensor_pose_cov_x': 1e-4, 'sensor_pose_cov_y': 1e-4, 'sensor_pose_cov_yaw': 1e-4}


def launch_setup(context):
    arg = lambda k: LaunchConfiguration(k).perform(context)
    experiment_filename = arg('experiment')
    start_idle = arg('start_idle').lower() == 'true'
    record = arg('record').lower() == 'true'

    common = os.path.join(get_package_share_directory('holohover_utils'), 'config', 'common')
    navigation_config = os.path.join(common, 'navigation_config.yaml')
    ecp = ecp_lib.ExperimentConfigParser(experiment_filename)
    sim_config = ecp.getSimConfig()
    if any(sim_config['simulated']):
        raise RuntimeError(f"{experiment_filename}: every hovercraft must be real (simulate: false)")
    print(f"Air hockey on the real table, experiment {experiment_filename}")

    ld = [
        Node(
            package='holohover_utils',
            executable='optitrack_interface',
            parameters=[os.path.join(common, 'simulation_config.yaml'), sim_config,
                        {'table_center': ecp.data['experiment'].get('table_center', [0.0, 0.0])}],
            output='screen',
        ),
        Node(
            package='holohover_navigation',
            executable='navigation',
            namespace='puck',
            parameters=[navigation_config, PUCK_EKF],
            remappings=[('pose', '/puck/pose')],
            ros_arguments=['--disable-rosout-logs'],
            output='screen',
        ),
    ]

    hovercraft = ecp.data['hovercraft'][:len(sim_config['simulated'])]
    controllers = [hc.get('controller') or {'type': 'plan'} for hc in hovercraft]
    folder = os.path.join('/root/data', arg('name') or ecp.data['experiment']['name'])
    run = len(os.listdir(folder)) + 1 if os.path.isdir(folder) else 1
    bag = os.path.join(folder, f"{run:03d}_{datetime.now():%Y%m%d_%H%M%S}")
    if record:
        print(f"Recording every topic to {bag}")
        os.makedirs(folder, exist_ok=True)
        ld.append(ExecuteProcess(cmd=['ros2', 'bag', 'record', '-a', '-s', 'mcap', '-o', bag], output='screen'))

    _, names, props_files, _ = ecp.getHovercraft()
    for name, props, hc, ctrl in zip(names, props_files, hovercraft, controllers):
        if ctrl['type'] not in ('plan', 'lqr'):
            raise RuntimeError(f"{name}: controller type {ctrl['type']!r}, expected plan or lqr")
        if ctrl['type'] == 'lqr':
            # the hovercraft's EKF (navigation_disturbance) and its LQR
            ld.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(this_dir, 'hovercraft_lqr.launch.py')),
                launch_arguments={'name': name, 'params': props, 'initial_x': str(ctrl.get('x', 0.0)),
                                  'initial_y': str(ctrl.get('y', 0.0)),
                                  'initial_yaw': str(ctrl.get('yaw', 0.0))}.items(),
            ))
            continue
        ld += [
            Node(
                package='holohover_navigation',
                executable='navigation_disturbance',
                namespace=name,
                parameters=[navigation_config, {'holohover_props_file': props}],
                ros_arguments=['--disable-rosout-logs'],
                output='screen',
            ),
            Node(
                package='holohover_control',
                executable='control_plan',
                namespace=name,
                parameters=[os.path.join(common, 'control_plan_config.yaml'),
                            os.path.join(common, 'control_plan_real.yaml'),  # real-robot overrides (defence PD)
                            {'holohover_props_file': props, 'start_idle': start_idle}],
                output='screen',
                emulate_tty=True,
                additional_env={'PYTHONUNBUFFERED': '1'},
            ),
        ]
        if (hc.get('planner') or {}).get('active', True):
            ld.append(Node(
                package='holohover_planner',
                executable='shot_planner',
                namespace=name,
                parameters=[os.path.join(common, 'shot_planner_config.yaml'), {'start_idle': start_idle}],
                output='screen',
                emulate_tty=True,
                additional_env={'PYTHONUNBUFFERED': '1'},
            ))
    return ld


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('experiment', default_value='air_hockey_real.yaml', description='Experiment file'),
        DeclareLaunchArgument('start_idle', default_value='true', choices=['true', 'false'],
                              description='Planner and controller idle until an episode >= 0 on hN/episode'),
        DeclareLaunchArgument('record', default_value='false', choices=['true', 'false'],
                              description='Record every topic to an MCAP bag'),
        DeclareLaunchArgument('name', default_value='',
                              description="Bag folder under /root/data (default: the experiment's name)"),
        OpaqueFunction(function=launch_setup),
    ])
