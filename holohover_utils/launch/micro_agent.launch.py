import os
import launch.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()


    
    micro_agent = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        arguments=["udp4", "-p", "8888"],
        output='screen'
    )
    

    ld.add_action(micro_agent)

    return ld
