from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    simulator_node = Node(
        package="base_package",
        executable="simulator",
    )

    controller_node = Node(
        package="base_package",
        executable="controller"
    )

    estimator_node = Node(
        package="base_package",
        executable="estimator"
    )
    ld.add_action(simulator_node)
    ld.add_action(estimator_node)
    ld.add_action(controller_node)
    return ld