from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    simulator_node = Node(
        package="holohover_gnc",
        executable="simulator",
    )

    controller_node = Node(
        package="holohover_gnc",
        executable="controller"
    )

    estimator_node = Node(
        package="holohover_gnc",
        executable="estimator"
    )

    ld.add_action(simulator_node)
    ld.add_action(estimator_node)
    ld.add_action(controller_node)
    return ld
