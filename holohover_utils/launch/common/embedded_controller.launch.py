import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import yaml

def launch_setup(context):
    experiment_filename = LaunchConfiguration('experiment').perform(context)
    machine = LaunchConfiguration('machine').perform(context)
    name = ''
    launch_description = []

    this_dir = os.path.dirname(os.path.abspath(__file__))
    
    experiment_conf = os.path.join(
        get_package_share_directory('holohover_utils'), 
        'config', 
        'experiments', 
        experiment_filename
    )

    data = yaml.safe_load(open(experiment_conf, 'r'))
    hovercraft = data["hovercraft"]
     
    for h in hovercraft:
        if h['machine'] == machine:
            name = h['name']
            break
    
    if name == '':
        print("Not starting any controller")
        return launch_description

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
        parameters=[drivers_config],
        output='both'
    )

    mouse_node = Node(
        name="holohover_mouse_sensor",
        package="holohover_drivers",
        executable="holohover_mouse_sensor",
        namespace=name,
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
        'experiment', default_value='experiment1.yaml',
        description='Experiment File'
    ))

    ld.add_action(DeclareLaunchArgument(
        'machine', default_value='master',
        description='Machine Name'
    ))

    ld.add_action(opfunc)
 
    return ld