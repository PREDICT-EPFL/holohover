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
    # ToDo: add parameter for experiment file
    experiment_filename = "experiment1.yaml"
    
    experiment_conf = os.path.join(
        get_package_share_directory('holohover_utils'), 
        'config', 
        'experiments', 
        experiment_filename
    )

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

    navigation_config = os.path.join(
        get_package_share_directory('holohover_utils'),
        'config/common',
        'navigation_config.yaml'
    )

    data = yaml.safe_load(open(experiment_conf, 'r'))

    print(f" - - - - STARTING SIMULATION EXPERIMENT  - - - - ")
    print(f"Running experiment:\t\t{data['experiment']['name']}")
    print(f"Experiment description:\t\t{data['experiment']['description']}")

    # Start common nodes
    simulation_env_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(this_dir, 'common/simulation_env.launch.py'))
    )
    #ld.add_action(simulation_env_launch)

    # Now iterate on each hovercraft and launch the nodes for each one
    hovercrafts = data["hovercrafts"]

    print(f"Starting {len(hovercrafts)} hovercrafts")
    for hovercraft_conf in hovercrafts:
        print(f"\t- hovercraft\t\tID: {hovercraft_conf['id']} - Name: {hovercraft_conf['name']} - Initial state: {hovercraft_conf['initial_state']} ")
        print(hovercraft_conf)
        namespace = hovercraft_conf['name']

        specific_configuration = os.path.join(
            get_package_share_directory('holohover_utils'),
            'config/hovercrafts',
            namespace,
            'config.yaml'
        )

        navigation_node = Node(
            package="holohover_gnc",
            executable="navigation",
            parameters=[holohover_params, navigation_config],
            namespace= namespace,
            output='screen'
        )

        controller_node = Node(
            package="holohover_gnc",
            executable="control_lqr",
            parameters=[holohover_params, control_lqr_config],
            name = "control_lqr",
            namespace= namespace,
            output='screen'
        )
        
        rviz_interface_node = Node(
            package="holohover_utils",
            executable="rviz_interface",
            parameters=[holohover_params, specific_configuration, hovercraft_conf],
            namespace= namespace,
            output='screen'
        )

        simulation_config = os.path.join(
            get_package_share_directory('holohover_utils'),
            'config/common',
            'simulation_config.yaml'
        )
        simulation_node = Node(
            package="holohover_gnc",
            executable="simulation",
            namespace= namespace,
            parameters=[holohover_params, simulation_config],
            output='screen'
        )

        ld.add_action(controller_node)        
        ld.add_action(navigation_node)
        ld.add_action(rviz_interface_node)
        ld.add_action(simulation_node)

        #ld.add_action(hovercraft_launch)

    return ld
