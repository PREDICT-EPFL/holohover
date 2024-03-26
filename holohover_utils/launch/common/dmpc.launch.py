import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    holohover_params = os.path.join(
        get_package_share_directory('holohover_utils'),
        'config',
        'common',
        'holohover_params.yaml'
    )

    control_dmpc_config0 = os.path.join(
        get_package_share_directory('holohover_dmpc'),
        'config',
        'control_dmpc_config0.yaml'
    )

    control_dmpc_config1 = os.path.join(
        get_package_share_directory('holohover_dmpc'),
        'config',
        'control_dmpc_config1.yaml'
    )

    control_dmpc_config2 = os.path.join(
        get_package_share_directory('holohover_dmpc'),
        'config',
        'control_dmpc_config2.yaml'
    )

    control_dmpc_config3 = os.path.join(
        get_package_share_directory('holohover_dmpc'),
        'config',
        'control_dmpc_config3.yaml'
    )

    dmpc_trigger_config = os.path.join(
        get_package_share_directory('holohover_dmpc'),
        'config',
        'dmpc_trigger_config.yaml'
    )

    controller_node0 = Node(
        name="dmpc",
        package="holohover_dmpc",
        executable="control_dmpc",
        parameters=[control_dmpc_config0,
        {"holohover_props_file": holohover_params}],
        namespace="h0",
        output='both'
    )

    controller_node1 = Node(
        name="dmpc",
        package="holohover_dmpc",
        executable="control_dmpc",
        parameters=[control_dmpc_config1,
        {"holohover_props_file": holohover_params}],
        namespace="h1",
        output='both'
    )

    controller_node2 = Node(
        name="dmpc",
        package="holohover_dmpc",
        executable="control_dmpc",
        parameters=[control_dmpc_config2,
        {"holohover_props_file": holohover_params}],
        namespace="h2",
        output='both'
    )

    controller_node3 = Node(
        name="dmpc",
        package="holohover_dmpc",
        executable="control_dmpc",
        parameters=[control_dmpc_config3,
        {"holohover_props_file": holohover_params}],
        namespace="h3",
        output='both'
    )

    trigger_node = Node(
        name="holohover_dmpc_trigger",
        package="holohover_dmpc",
        executable="dmpc_trigger",
        parameters=[dmpc_trigger_config],
        output='both'
    )

    ld.add_action(controller_node0)
    ld.add_action(controller_node1)
    ld.add_action(controller_node2)
    ld.add_action(controller_node3)
    ld.add_action(trigger_node)

    return ld