import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    trajectory_config = os.path.join(
        get_package_share_directory('holohover_dmpc'),
        'config/trajectories',
        'trajectory1.yaml'
    )

    trajectory_delay_arg = DeclareLaunchArgument(
        'trajectory_delay', default_value='0.0',
        description='Delay before start recording'
    )

    trajectory_node = Node(
        package="holohover_dmpc",
        executable="trajectory_generator",
        parameters=[{"trajectory_file": trajectory_config}] ,
        output='screen'
    )

    trajectory_node_delayed = TimerAction(
        period=LaunchConfiguration("trajectory_delay"),
        actions=[
            trajectory_node,
        ] 
    )

    ld.add_action(trajectory_delay_arg)
    ld.add_action(trajectory_node_delayed)

    return ld


