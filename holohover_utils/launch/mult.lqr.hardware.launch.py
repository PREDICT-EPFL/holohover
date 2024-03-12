import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node
import launch_ros.actions

import yaml

def generate_launch_description():
    ld = LaunchDescription()

    this_dir = os.path.dirname(os.path.abspath(__file__))

    #ld.add_action(DeclareLaunchArgument('experiment', default_value='experiment1.yaml'))
    #experiment = ParameterValue(LaunchConfiguration('experiment')).get_parameter_value().string_value
    #experiment = LaunchConfiguration('experiment', default='experiment1.yaml')

    experiment_filename = "experiment1.yaml"
    
    experiment_conf = os.path.join(
        get_package_share_directory('holohover_utils'),
        'config',
        'experiments',
        experiment_filename
        )
    
    with open(experiment_conf, "r") as file:
        data = yaml.safe_load(file)

    print(f"Running experiment:\t\t{data['experiment']['name']}")
    print(f"Experiment description:\t\t{data['experiment']['description']}")

    hardware_env_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(this_dir, 'common/hardware_env.launch.py'))
    )
    ld.add_action(hardware_env_launch)
    hovercrafts = data["hovercrafts"]

    print(f"Starting {len(hovercrafts)} hovercrafts")
    for hovercraft_conf in hovercrafts:
        print(f"\t- hovercraft\t\tID: {hovercraft_conf['id']} - Name: {hovercraft_conf['name']} - Type: {hovercraft_conf['type']}")

        #namespace = hovercraft_conf['id']

        hovercraft_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(this_dir, 'common/hovercraft.launch.py')),
            #launch_arguments={'namespace': namespace, 'conf': hovercraft_conf}.items()
        )


        #visualization_launch = IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(os.path.join(this_dir, 'visualization.launch.py'))
        #)
        
        holohover_params = os.path.join(
            get_package_share_directory('holohover_utils'),
            'config/common',
            'holohover_params.yaml'
        )

        control_lqr_config = os.path.join(
            get_package_share_directory('holohover_utils'),
            'config/common',
            'control_lqr_config.yaml'
        )
        
        controller_node = Node(
            package="holohover_gnc",
            executable="control_lqr",
            parameters=[holohover_params, control_lqr_config, {}],
            name = "control_lqr",
            namespace=("eddai"+str(hovercraft_conf['id'])),
            output='screen'
        )
        ld.add_action(controller_node)
        #ld.add_action(hovercraft_launch)

    return ld
