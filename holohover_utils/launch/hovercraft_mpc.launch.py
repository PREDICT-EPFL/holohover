from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def launch_setup(context):
    launch_description = []

    index  = LaunchConfiguration('index').perform(context)
    name   = LaunchConfiguration('name').perform(context)
    params = LaunchConfiguration('params').perform(context)
    initial_x = float(LaunchConfiguration('initial_x').perform(context))    
    initial_y = float(LaunchConfiguration('initial_y').perform(context))    
    initial_yaw = float(LaunchConfiguration('initial_yaw').perform(context))    
    
    print(f"\t- hovercraft\t\tID: {index} - Name: {name}")
    print(f"Configuration file: {params}")
    
    control_mpc_config = os.path.join(
        get_package_share_directory('holohover_utils'),
        'config',
        'common',
        'control_mpc_config.yaml'
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
        ros_arguments=['--disable-rosout-logs'],
        output='screen',
    )
    
    controller_node = Node(
        name="mpc",
        package="holohover_mpc",
        executable="control_mpc",
        parameters=[control_mpc_config,
        {"holohover_props_file": params, "initial_x": initial_x, "initial_y": initial_y, "initial_yaw": initial_yaw}],
        namespace=name,
        ros_arguments=['--disable-rosout-logs'],
        output='both',
    )
    
    this_dir = os.path.dirname(os.path.abspath(__file__))

    recorder_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(this_dir, 'common/recorder_specific.launch.py')),
                launch_arguments=
                    {'name': name}.items()
            )

    launch_description.append(controller_node)        
    launch_description.append(navigation_node)
    # launch_description.append(recorder_launch)

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
        'params', default_value='/root/ros2_ws/install/holohover_utils/share/holohover_utils/config/common/holohover_params.yaml',
        description='Holohover params config file path.'
    ))

    ld.add_action(DeclareLaunchArgument(
        'initial_x', default_value='0.0',
        description='Initial X position.'
    ))

    ld.add_action(DeclareLaunchArgument(
        'initial_y', default_value='0.0',
        description='Initial Y position.'
    ))

    ld.add_action(DeclareLaunchArgument(
        'initial_yaw', default_value='0.0',
        description='Initial Yaw angle.'
    ))

    ld.add_action(opfunc)   

    return ld