"""
Velocity teleop of one hovercraft: a gamepad or the keyboard publishes a Twist on hN/cmd_vel, holohover_control
teleop turns it into a moving reference (hN/state_ref) that control_lqr tracks.

    ros2 launch holohover_utils teleop.launch.py                                   # sim, PS5 gamepad, h0
    ros2 launch holohover_utils teleop.launch.py input:=keyboard
    ros2 launch holohover_utils teleop.launch.py robot:=real name:=h3

robot: sim (default) starts air_hockey_sim.launch.py controller:=teleop (Box2D simulator, EKF, control_lqr and
teleop per hovercraft of its experiment); `name` must be one of them. real starts, for `name`, its EKF and
control_lqr (hovercraft_lqr.launch.py) and teleop; hw_env.launch.py (OptiTrack) and the hovercraft's own
launchFC_loc.sh run separately.
input: ps5 (default): joy + teleop_twist_joy (config/common/teleop_joy_config.yaml: hold L1, left stick x/y,
right stick yaw, R1 turbo). keyboard: teleop_twist_keyboard in an xterm (needs a display; over ssh, ssh -X).
pose_receive_time (real only, default true): the EKF times poses on arrival, the mocap PC's clock is not synced.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    arg = lambda k: LaunchConfiguration(k).perform(context)
    robot, keyboard, name = arg('robot'), arg('input') == 'keyboard', arg('name')
    this_dir = os.path.dirname(os.path.abspath(__file__))
    config = os.path.join(get_package_share_directory('holohover_utils'), 'config')
    cmd_vel = [('cmd_vel', f'/{name}/cmd_vel')]

    if robot == 'sim':
        ld = [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(this_dir, 'air_hockey_sim.launch.py')),
            launch_arguments={'controller': 'teleop'}.items())]
    else:
        ld = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(this_dir, 'hovercraft_lqr.launch.py')),
                launch_arguments={'index': name.lstrip('h'), 'name': name,
                                  'params': os.path.join(config, arg('params')),
                                  'pose_receive_time': arg('pose_receive_time')}.items()),
            Node(package='holohover_control', executable='teleop', namespace=name, output='screen',
                 emulate_tty=True, additional_env={'PYTHONUNBUFFERED': '1'}),
        ]

    if keyboard:
        ld.append(Node(package='teleop_twist_keyboard', executable='teleop_twist_keyboard', prefix='xterm -e',
                       remappings=cmd_vel, output='screen'))
    else:
        joy_config = os.path.join(config, 'common', 'teleop_joy_config.yaml')
        ld += [
            Node(package='joy', executable='game_controller_node', parameters=[joy_config], output='screen'),
            Node(package='teleop_twist_joy', executable='teleop_node', parameters=[joy_config],
                 remappings=cmd_vel, output='screen'),
        ]
    return ld


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='sim', choices=['sim', 'real']),
        DeclareLaunchArgument('input', default_value='ps5', choices=['ps5', 'keyboard']),
        DeclareLaunchArgument('name', default_value='h0', description='Hovercraft driven (real robot: e.g. h3)'),
        DeclareLaunchArgument('params', default_value='common/holohover_params_new.yaml',
                              description='Real robot: props file, from holohover_utils/config'),
        DeclareLaunchArgument('pose_receive_time', default_value='true', choices=['true', 'false'],
                              description='Real robot: EKF times poses on arrival (mocap clock not synced)'),
        OpaqueFunction(function=launch_setup),
    ])
