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
    dmpc_config_folder = LaunchConfiguration('dmpc_config_folder').perform(context)
    
    file_name_xd_trajectory = LaunchConfiguration('file_name_xd_trajectory').perform(context) if index == "0" else ""
    file_name_ud_trajectory = LaunchConfiguration('file_name_ud_trajectory').perform(context)

    obstacles = LaunchConfiguration('obstacles').perform(context)

    print(f"\t- hovercraft\t\tID: {index} - Name: {name}")
    print(f"\t - xd trajectory file: {file_name_xd_trajectory}")
    print(f"Configuration file: {params}")
    
    control_dmpc_config = os.path.join(
        get_package_share_directory('holohover_dmpc'),
        'config',
        dmpc_config_folder,
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
        executable="navigation_disturbance",    
        parameters=[navigation_config, {'holohover_props_file' : params}],
        namespace= name,
        output='screen',
        prefix='nice -n -19'
    )
    
    controller_node = Node(
        name="dmpc",
        package="holohover_dmpc",
        executable="control_dmpc_" + opt_alg,
        parameters=[control_dmpc_config,
        {"holohover_props_file": params, 'file_name_xd_trajectory': file_name_xd_trajectory, 'file_name_ud_trajectory': file_name_ud_trajectory, 'obstacles': obstacles.split('---')}],
        namespace=name,
        output='screen',
        #prefix='gdb -ex run --args',
        prefix='nice -n -19',
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