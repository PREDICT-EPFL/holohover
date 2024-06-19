import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    drivers_config = os.path.join(
        get_package_share_directory('holohover_utils'),
        'config',
        'common',
        'drivers_config.yaml'
    )

    fc_node = Node(
        name="holohover_fc",
        package="holohover_drivers",
        executable="holohover_fc",
        parameters=[drivers_config],
        output='both'
    )

    mouse_node = Node(
        name="holohover_mouse_sensor",
        package="holohover_drivers",
        executable="holohover_mouse_sensor",
        parameters=[drivers_config],
        output='both'
    )

    ld.add_action(fc_node)
    #ld.add_action(mouse_node)

    return ld