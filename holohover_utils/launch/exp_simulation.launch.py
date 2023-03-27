import os
import launch.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    holohover_params = os.path.join(
        get_package_share_directory('holohover_gnc'),
        'config',
        'holohover_params.yaml'
    )

    simulation_config = os.path.join(
        get_package_share_directory('holohover_gnc'),
        'config',
        'simulation_config.yaml'
    )
    simulation_node = Node(
        package="holohover_gnc",
        executable="simulation",
        parameters=[holohover_params, simulation_config],
        output='screen'
    )

    navigation_config = os.path.join(
        get_package_share_directory('holohover_gnc'),
        'config',
        'navigation_config.yaml'
    )
    navigation_node = Node(
        package="holohover_gnc",
        executable="navigation",
        parameters=[holohover_params, navigation_config],
        output='screen'
    )

    control_lqr_config = os.path.join(
        get_package_share_directory('holohover_gnc'),
        'config',
        'control_lqr_config.yaml'
    )


    controller_node = Node(
        package="holohover_gnc",
        executable="control_exp",
        parameters=[holohover_params, control_lqr_config],
        output='screen'
    )

    rviz_interface_node = Node(
        package="holohover_utils",
        executable="rviz_interface",
        parameters=[holohover_params],
        output='screen'
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
        output='screen'
    )
    
    recorder = launch.actions.ExecuteProcess(
    	cmd=['ros2', 'bag', 'record', '-a'],
    	output='screen'
    )

    ld.add_action(simulation_node)
    ld.add_action(navigation_node)
    ld.add_action(controller_node)
    ld.add_action(rviz_interface_node)
    ld.add_action(rviz_node)
    ld.add_action(recorder)

    return ld
