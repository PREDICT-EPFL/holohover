import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

this_dir = os.path.abspath(os.path.join(os.path.dirname(__file__),".."))
if this_dir not in sys.path:
    sys.path.insert(0, this_dir)

import experiment_config_parser as ecp_lib

def launch_setup(context):
    ld = []

    experiment_filename = LaunchConfiguration('experiment').perform(context)
    machine = LaunchConfiguration('machine').perform(context)

    print(f"Launching experiment from file: {experiment_filename}")
    print(f"This machine is named: {machine}")
    
    ecp = ecp_lib.ExperimentConfigParser(experiment_filename)
    
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

    simulator_config = os.path.join(
        get_package_share_directory('holohover_utils'),
        'config/common',
        'simulation_config.yaml'
    )

    rviz_interface_node = Node(
        package="holohover_utils",
        executable="rviz_interface",
        parameters=[simulator_config,
                    ecp.getSimConfig()],
        output='screen'
    )

    if ecp.getCommonNodesMachine() == machine or machine == "all":
        ld.append(rviz_node)
        ld.append(rviz_interface_node)

    return ld

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
