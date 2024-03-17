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

    control_dmpc_config0 = os.path.join(
        get_package_share_directory('holohover_gnc'),
        'config',
        'control_dmpc_config0.yaml'
    )

    control_dmpc_config1 = os.path.join(
        get_package_share_directory('holohover_gnc'),
        'config',
        'control_dmpc_config1.yaml'
    )

    control_dmpc_config2 = os.path.join(
        get_package_share_directory('holohover_gnc'),
        'config',
        'control_dmpc_config2.yaml'
    )

    control_dmpc_config3 = os.path.join(
        get_package_share_directory('holohover_gnc'),
        'config',
        'control_dmpc_config3.yaml'
    )

    controller_node0 = Node(
        package="holohover_gnc",
        executable="control_dmpc",
        parameters=[holohover_params, control_dmpc_config0],
        output='screen'
    )

    controller_node1 = Node(
        package="holohover_gnc",
        executable="control_dmpc",
        parameters=[holohover_params, control_dmpc_config1],
        output='screen'
    )

    controller_node2 = Node(
        package="holohover_gnc",
        executable="control_dmpc",
        parameters=[holohover_params, control_dmpc_config2],
        output='screen'
    )

    controller_node3 = Node(
        package="holohover_gnc",
        executable="control_dmpc",
        parameters=[holohover_params, control_dmpc_config3],
        output='screen'
    )

    ld.add_action(controller_node0)
    ld.add_action(controller_node1)
    ld.add_action(controller_node2)
    ld.add_action(controller_node3)

    return ld