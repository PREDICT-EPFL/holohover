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
    """
    Generate launch description for holohover_dial nodes.
    
    This launch file:
    1. Declares a launch argument for the parameter file
    2. Starts the path planner node
    3. Starts the control node
    4. Both nodes share the same parameters
    """
    
    # Get the package directory
    holohover_dial_dir = get_package_share_directory('holohover_dial')
    default_params_file = os.path.join(holohover_dial_dir, 'example_params.yaml')
    
    # Declare launch argument for params file
    declare_params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to parameter file to load'
    )
    
    # Path Planner Node
    # - Subscribes to /holohover/state
    # - Publishes to /holohover/path_plan
    path_planner_node = Node(
        package='holohover_dial',
        executable='path_planner',
        name='path_planner',
        parameters=[LaunchConfiguration('params_file')],
        output='screen',  # Show logs in terminal
        emulate_tty=True,  # Colorize output
    )
    
    # Control Node
    # - Subscribes to /holohover/state and /holohover/path_plan
    # - Publishes to /holohover/control
    control_node = Node(
        package='holohover_dial',
        executable='control',
        name='control_node',
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        declare_params_arg,
        path_planner_node,
        control_node,
    ])
