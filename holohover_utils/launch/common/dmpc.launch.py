import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    dmpc_trigger_config = os.path.join(
        get_package_share_directory('holohover_dmpc'),
        'config',
        'dmpc_trigger_config.yaml'
    )

    trigger_node = Node(
        name="holohover_dmpc_trigger",
        package="holohover_dmpc",
        executable="dmpc_trigger",
        parameters=[dmpc_trigger_config],
        output='both'
    )

    ld.add_action(trigger_node)

    return ld