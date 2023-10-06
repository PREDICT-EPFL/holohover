import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    launch_arg = DeclareLaunchArgument(
        'exp_delay', default_value='0.0',
        description='Delay before starting the experiment'
    )

    holohover_params = os.path.join(
        get_package_share_directory('holohover_common'),
        'config',
        'holohover_params.yaml'
    )

    control_lqr_config = os.path.join(
        get_package_share_directory('holohover_gnc'),
        'config',
        'control_lqr_config.yaml'
    )

    control_exp_config = os.path.join(
        get_package_share_directory('holohover_gnc'),
        'config',
        'control_exp_config.yaml'
    )

    controller_node = Node(
        package="holohover_gnc",
        executable="control_exp",
        parameters=[holohover_params, control_lqr_config, control_exp_config],
        output='screen'
    )

    controller_delayed = TimerAction(
        period=LaunchConfiguration('exp_delay'),
        actions=[
        	controller_node,
        ]
    )

    ld.add_action(launch_arg)
    ld.add_action(controller_delayed)

    return ld
