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
from ament_index_python.packages import get_package_share_directory


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




    #################### COMMON NODES STARTING ####################
    print(f" - - - - STARTING SIMULATION EXPERIMENT  - - - - ")
    
    exp_name, exp_desc = ecp.getExperimentNameDesc()
    print(f"Running experiment:\t\t{exp_name}")
    print(f"Experiment description:\t\t{exp_desc}")

    hovercraft_names, hovercraft_ids = ecp.getHovercraftNamesIds()

    ## Trajectory Generator
    trajectory_generator_node = Node(
        package="holohover_dmpc",
        executable="trajectory_generator",
        parameters=[{ "ids" :       hovercraft_ids, 
                      "names" :     hovercraft_names}],
        output='both',
        prefix='xterm -e'
    )

    ## DMPC Trigger
    trigger_node = Node(
        name="holohover_dmpc_trigger",
        package="holohover_dmpc",
        executable="dmpc_trigger",
        output='both'
    )

    if ecp.getCommonNodesMachine() == machine or machine == "all":
        launch_description.append(trigger_node)
        launch_description.append(trajectory_generator_node)

    #################### COMMON NODES STARTING - END ####################
   

    # Now iterate on each hovercraft and obstacleand launch the nodes for each one
    hovercraft_machines, hovercraft_names, hovercraft_params, _ = ecp.getHovercraft()
    opt_alg,file_name_xd_trajectory,file_name_ud_trajectory, dmpc_config_folder = ecp.getDMPCdata()
    obstacle_machines, obstacle_names, obstacle_params, obstacle_initial_states = ecp.getObstacles()

    #################### HOVERCRAFT STARTING ####################

    print(f"Starting {len(hovercraft_names)} hovercraft")
    
    for i in range(len(hovercraft_names)):
        if hovercraft_machines[i] == machine or machine == "all":
            hovercraft_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(this_dir, 'hovercraft_dmpc.launch.py')),
                launch_arguments={
                    'index': str(i),
                    'name': hovercraft_names[i],
                    'params': hovercraft_params[i],
                    'opt_alg': opt_alg,
                    'dmpc_config_folder': dmpc_config_folder,
                    'file_name_xd_trajectory': file_name_xd_trajectory,
                    'file_name_ud_trajectory': file_name_ud_trajectory,
                    'obstacles': '---'.join(obstacle_names),
                    }.items()
            )

            launch_description.append(hovercraft_launch)
    #################### HOVERCRAFT STARTING - END ####################

    #################### OBSTACLE STARTING ####################
    # Now iterate on each hovercraft and launch the nodes for each one

    print(f"Starting {len(obstacle_names)} obstacles")
    
    for i in range(len(obstacle_names)):
        if obstacle_machines[i] == machine or machine == "all":
            hovercraft_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(this_dir, 'hovercraft_lqr.launch.py')),
                launch_arguments={
                    'index': str(i + len(obstacle_names) - 1), 
                    'name': obstacle_names[i], 
                    'params': obstacle_params[i],
                    'initial_x': str(obstacle_initial_states['x'][i]),
                    'initial_y': str(obstacle_initial_states['y'][i]),
                    'initial_yaw': str(obstacle_initial_states['theta'][i])
                    }.items()
            )

            launch_description.append(hovercraft_launch)
    #################### OBSTACLE STARTING - END ####################

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

