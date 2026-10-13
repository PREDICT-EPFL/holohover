import os
import sys
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction

this_dir = os.path.abspath(os.path.dirname(__file__))
if this_dir not in sys.path:
    sys.path.insert(0, this_dir)

import experiment_config_parser as ecp_lib


def launch_setup(context):
    
    experiment_filename = LaunchConfiguration('experiment').perform(context)
    machine = LaunchConfiguration('machine').perform(context)

    print(f"Launching experiment from file: {experiment_filename}")
    print(f"This machine is named: {machine}")

    launch_description = []

    ecp = ecp_lib.ExperimentConfigParser(experiment_filename)

    print(f" - - - - STARTING SIMULATION EXPERIMENT  - - - - ")

    exp_name, exp_desc = ecp.getExperimentNameDesc()
    print(f"Running experiment:\t\t{exp_name}")
    print(f"Experiment description:\t\t{exp_desc}")

    simulator_config = os.path.join(
        get_package_share_directory('holohover_utils'),
        'config/common',
        'simulation_config.yaml'
    )

    simulator_node = Node(
        package="holohover_simulator",
        executable="simulator",
        parameters=[simulator_config,
                    ecp.getSimConfig()],
        output='screen'
    )

    optitrack_node = Node(
        package="holohover_utils",
        executable="optitrack_interface",
        parameters=[simulator_config,
                    ecp.getSimConfig()],
        output='screen'
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(this_dir, 'common', 'rviz.launch.py')),
        launch_arguments=
            {'experiment': experiment_filename,
             'machine': machine}.items()
    )

    recorder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(this_dir, 'common', 'recorder.launch.py'))
    )
    
    if ecp.getCommonNodesMachine() == machine or machine == "all":
        launch_description.append(rviz_launch)
        if any(ecp.getSimConfig()["simulated"]): 
           launch_description.append(simulator_node)
        launch_description.append(optitrack_node)
        launch_description.append(recorder_launch)
        
        
    #################### COMMON NODES STARTING - END ####################
   

    return launch_description


def generate_launch_description():
    ld = LaunchDescription()

    opfunc = OpaqueFunction(function = launch_setup)

    ld.add_action(DeclareLaunchArgument(
        'experiment', default_value='experiment1.yaml',
        description='Experiment File'
    ))

    ld.add_action(DeclareLaunchArgument(
        'machine', default_value='master',
        description='Machine Name'
    ))

    ld.add_action(opfunc)
 
    return ld

