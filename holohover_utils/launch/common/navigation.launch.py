import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    holohover_params = os.path.join(
        get_package_share_directory('holohover_common'),
        'config',
        'holohover_params.yaml'
    )

    navigation_config = os.path.join(
        get_package_share_directory('holohover_gnc'),
        'config',
        'navigation_config.yaml'
    )

    navigation_node = Node(
        package="holohover_gnc",
        executable="navigation",
        parameters=[holohover_params, navigation_config],
        output='screen'
    )

    ld.add_action(navigation_node)

    return ld
