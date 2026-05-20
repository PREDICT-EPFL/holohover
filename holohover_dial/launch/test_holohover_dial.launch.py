"""
Example launch file for holohover_dial nodes.

This demonstrates how to launch both the path planner and control nodes together,
with shared parameters loaded from a YAML file.

Usage:
    ros2 launch holohover_dial holohover_dial.launch.py
    
Or with custom parameter file:
    ros2 launch holohover_dial holohover_dial.launch.py params_file:=/path/to/params.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    holohover_dial_dir = get_package_share_directory('holohover_dial')
    default_params_file = os.path.join(holohover_dial_dir, 'example_params.yaml')
    
    declare_params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to parameter file to load'
    )
    
    # Path Planner Node for Robot 1 (h1)
    path_planner_h1 = Node(
        package='holohover_dial',
        executable='path_planner',
        name='path_planner',
        namespace='h1',  # <-- Key: namespace for h1
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
        emulate_tty=True,
    )
    
    # Path Planner Node for Robot 2 (h2)
    path_planner_h2 = Node(
        package='holohover_dial',
        executable='path_planner',
        name='path_planner',
        namespace='h2',  # <-- Key: namespace for h2
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
        emulate_tty=True,
    )
    
    # Control Node for Robot 1 (h1)
    control_node_h1 = Node(
        package='holohover_dial',
        executable='control',
        name='control_node',
        namespace='h1',
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
        emulate_tty=True,
    )
    
    # Control Node for Robot 2 (h2)
    control_node_h2 = Node(
        package='holohover_dial',
        executable='control',
        name='control_node',
        namespace='h2',
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        declare_params_arg,
        path_planner_h1,
        path_planner_h2,
        control_node_h1,
        control_node_h2,
    ])
