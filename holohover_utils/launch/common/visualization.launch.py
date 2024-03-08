import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    #holohover_params = os.path.join(
    #    get_package_share_directory('holohover_utils'),
    #    'config/common',
    #    'holohover_params.yaml'
    #)

    #rviz_interface_node = Node(
    #    package="holohover_utils",
    #    executable="rviz_interface",
    #    parameters=[holohover_params],
    #    output='screen'
    #)

    rviz_config = os.path.join(
        get_package_share_directory('holohover_utils'),
        'gui',
        'holohover_config.rviz'
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config],
        output='screen'
    )

    #ld.add_action(rviz_interface_node)
    #ld.add_action(rviz_node)

    return ld
