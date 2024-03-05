import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    this_dir = os.path.dirname(os.path.abspath(__file__))

    holohover_params = os.path.join(
        get_package_share_directory('holohover_common'),
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

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(this_dir, 'navigation.launch.py'))
    )

    visualization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(this_dir, 'visualization.launch.py'))
    )

    recorder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(this_dir, 'recorder.launch.py'))
    )

    ld.add_action(simulation_node)
    ld.add_action(navigation_launch)
    ld.add_action(visualization_launch)
    ld.add_action(recorder_launch)

    return ld
