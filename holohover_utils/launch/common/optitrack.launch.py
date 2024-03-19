import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    mocap_config = os.path.join(
        get_package_share_directory('holohover_utils'),
        'config',
        'optitrack',
        'mocap_config.yaml'
    )

    mocap_node = Node(
        package='mocap_optitrack',
        executable='mocap_node',
        parameters=[mocap_config],
        output='screen'
    )

    optitrack_interface_node = Node(
        package="holohover_utils",
        executable="optitrack_interface",
        output='screen'
    )
    ld.add_action(mocap_node)
    ld.add_action(optitrack_interface_node)

    return ld
