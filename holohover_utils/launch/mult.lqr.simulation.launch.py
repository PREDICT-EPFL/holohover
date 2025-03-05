import os
import sys
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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

    #################### HOVERCRAFT STARTING ####################
    # Now iterate on each hovercraft and launch the nodes for each one
    hovercraft_machines, hovercraft_names, hovercraft_params, hovercraft_initial_states = ecp.getHovercraft()

    print(f"Starting {len(hovercraft_names)} hovercraft")
    
    for i in range(len(hovercraft_names)):
        if hovercraft_machines[i] == machine or machine == "all":
            hovercraft_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(this_dir, 'hovercraft_lqr.launch.py')),
                launch_arguments={
                    'index': str(i), 
                    'name': hovercraft_names[i], 
                    'params': hovercraft_params[i],
                    'initial_x': str(hovercraft_initial_states['x'][i]),
                    'initial_y': str(hovercraft_initial_states['y'][i]),
                    'initial_yaw': str(hovercraft_initial_states['theta'][i])}.items()
            )

            launch_description.append(hovercraft_launch)
    #################### HOVERCRAFT STARTING - END ####################


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

