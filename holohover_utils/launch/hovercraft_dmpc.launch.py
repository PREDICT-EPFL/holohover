from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context):
    launch_description = []

    index  = LaunchConfiguration('index').perform(context)
    name   = LaunchConfiguration('name').perform(context)
    params = LaunchConfiguration('params').perform(context)
    opt_alg = LaunchConfiguration('opt_alg').perform(context) #admm or dsqp
    
    print(f"\t- hovercraft\t\tID: {index} - Name: {name}")
    print(f"Configuration file: {params}")
    
    control_dmpc_config = os.path.join(
        get_package_share_directory('holohover_dmpc'),
        'config',
        'N10pi',
        'control_dmpc_config' + str(index) + '.yaml'
    )

    navigation_config = os.path.join(
        get_package_share_directory('holohover_utils'),
        'config/common',
        'navigation_config.yaml'
    )

    # - - - Nodes
    navigation_node = Node(
        package="holohover_navigation",
        executable="navigation",
        parameters=[navigation_config, {'holohover_props_file' : params}],
        namespace= name,
        output='screen'
    )
    
    controller_node = Node(
        name="dmpc",
        package="holohover_dmpc",
        executable="control_dmpc_" + opt_alg,
        parameters=[control_dmpc_config,
        {"holohover_props_file": params}],
        namespace=name,
        output='both',
        prefix='nice -n -19'
    )
    
    launch_description.append(controller_node)        
    launch_description.append(navigation_node)
    
    return launch_description


def generate_launch_description():
    ld = LaunchDescription()

    opfunc = OpaqueFunction(function = launch_setup)

    ld.add_action(DeclareLaunchArgument(
        'index', default_value='0',
        description='Hovercraft index.'
    ))

    ld.add_action(DeclareLaunchArgument(
        'name', default_value='h0',
        description='Hovercraft name.'
    ))

    ld.add_action(DeclareLaunchArgument(
        'params', default_value='common/holohover_params.yaml',
        description='Holohover params config file path.'
    ))


    ld.add_action(opfunc)   

    return ld