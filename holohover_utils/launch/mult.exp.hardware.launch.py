import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ParameterValue

import yaml

def generate_launch_description():
    ld = LaunchDescription()

    this_dir = os.path.dirname(os.path.abspath(__file__))

    #ld.add_action(DeclareLaunchArgument('experiment', default_value='experiment1.yaml'))
    #experiment = ParameterValue(LaunchConfiguration('experiment')).get_parameter_value().string_value

    #experiment = LaunchConfiguration('experiment', default='experiment1.yaml')
    experiment = "experiment1.yaml"


    hardware_env_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(this_dir, 'common/hardware_env.launch.py'))
    )

    experiment_conf = os.path.join(
        get_package_share_directory('holohover_utils'),
        'config',
        'experiments',
        experiment
        )

    with open(experiment_conf, "r") as file:
        data = yaml.safe_load(file)

    hovercrafts = data["hovercrafts"]
    world = data["world"]

    for hovercraft_conf in hovercrafts:
        print(f"Name: {hovercraft_conf['name']}")
        print(f"Id: {hovercraft_conf['id']}")
        print(f"Type: {hovercraft_conf['type']}")
        print(f"Graph Neighbours: {hovercraft_conf['graph_neighbours']}")
        print(f"Delta Pose: {hovercraft_conf['delta_pose']}")
        print(hovercraft_conf)

        namespace = hovercraft_conf['id']

        file_path = "/path/to/file.txt"

        if os.path.exists(file_path):
            print("File exists")
        else:
            print("File does not exist")

        hovercraft_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(this_dir, 'common/hovercraft.launch.py')),
            launch_arguments={'namespace': namespace, 'conf': hovercraft_conf}.items()
        )



    print("World:")
    print(f"Size: {world['size']}")

    


    exp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(this_dir, 'common/exp.launch.py'))
    )

    #ld.add_action(hardware_env_launch)
    #ld.add_action(exp_launch)

    return ld
