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

    control_mpc_config = os.path.join(
        get_package_share_directory('holohover_gnc'),
        'config',
        'control_mpc_config.yaml'
    )

    controller_node = Node(
        package="holohover_gnc",
        executable="control_mpc",
        parameters=[holohover_params, control_mpc_config],
        output='screen'
    )

    ld.add_action(controller_node)

    return ld
