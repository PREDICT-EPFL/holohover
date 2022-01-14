import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    
    camera_node = Node(
        package="holohover_gnc",
        executable="camera"
    )

    controller_node = Node(
        package="holohover_gnc",
        executable="controller"
    )

    estimator_node = Node(
        package="holohover_gnc",
        executable="estimator"
    )

    rviz_interface_node = Node(
        package="holohover_utils",
        executable="rviz_interface"
    )

    rviz_config = os.path.join(
        get_package_share_directory('holohover_utils'),
        'gui',
        'holohover_config.rviz'
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config],
    )

    ld.add_action(camera_node)
    ld.add_action(estimator_node)
    #ld.add_action(controller_node)
    ld.add_action(rviz_interface_node)
    ld.add_action(rviz_node)

    return ld
