import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import yaml

def launch_setup(context):
    launch_description = []
    name = LaunchConfiguration('name').perform(context)
    
    print("Starting controller for hovercraft", name)


    drivers_config = os.path.join(
        get_package_share_directory('holohover_utils'),
        'config',
        'common',
        'drivers_config.yaml'
    )

    fc_node = Node(
        name="holohover_fc",
        package="holohover_drivers",
        executable="holohover_fc",
        namespace=name,
        ros_arguments=['--disable-rosout-logs'],
        parameters=[drivers_config],
        output='both'
    )

    mouse_node = Node(
        name="holohover_mouse_sensor",
        package="holohover_drivers",
        executable="holohover_mouse_sensor",
        namespace=name,
        ros_arguments=['--disable-rosout-logs'],
        parameters=[drivers_config],
        output='both'
    )

    launch_description.append(fc_node)
    #launch_description.append(mouse_node)

    return launch_description




def generate_launch_description():
    ld = LaunchDescription()

    opfunc = OpaqueFunction(function = launch_setup)

    ld.add_action(DeclareLaunchArgument(
        'name', default_value='h0',
        description='Hovercraft name'
    ))

    ld.add_action(opfunc)
 
    return ld