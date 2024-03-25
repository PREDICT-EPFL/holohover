import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    rviz_config = os.path.join(
        get_package_share_directory('holohover_utils'),
        'gui',
        'holohover_config.rviz'
    )

    simulator_config = os.path.join(
        get_package_share_directory('holohover_utils'),
        'config/common',
        'simulation_config.yaml'
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config],
        output='screen'
    )

    rviz_wall_publisher_node = Node(
        package="holohover_utils",
        executable="rviz_wall_publisher",
        parameters=[simulator_config],
        output='screen'
    )

    ld.add_action(rviz_node)
    ld.add_action(rviz_wall_publisher_node)

    return ld
