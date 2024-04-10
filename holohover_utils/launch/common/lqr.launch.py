import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    holohover_params = os.path.join(
        get_package_share_directory('holohover_utils'),
        'config',
        'holohover_params.yaml'
    )

    control_lqr_config = os.path.join(
        get_package_share_directory('holohover_utils'),
        'config',
        'control_lqr_config.yaml'
    )

    controller_node = Node(
        package="holohover_control",
        executable="control_lqr",
        parameters=[holohover_params, control_lqr_config],
        output='screen'
    )

    ld.add_action(controller_node)

    return ld