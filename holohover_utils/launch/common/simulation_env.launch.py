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
        get_package_share_directory('holohover_utils'),
        'config/common',
        'holohover_params.yaml'
    )

    simulation_config = os.path.join(
        get_package_share_directory('holohover_utils'),
        'config/common',
        'simulation_config.yaml'
    )

    experiment_config = os.path.join(
        get_package_share_directory('holohover_utils'),
        'config/experiments',
        'experiment1.yaml' # ToDo: add parameter for experiment file
    )
    
    simulation_node = Node(
        package="holohover_utils",
        executable="simulator",
        parameters=[holohover_params, simulation_config, experiment_config], #ToDo simulation config from main config
        output='screen'
    )

    recorder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(this_dir, 'recorder.launch.py'))
    )

    visualization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(this_dir, 'visualization.launch.py'))
    )

    ld.add_action(simulation_node)
    ld.add_action(recorder_launch)
    ld.add_action(visualization_launch)

    return ld
