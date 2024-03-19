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

    # ToDo: add parameter for experiment file
    experiment_filename = "experiment1.yaml"
    
    experiment_conf = os.path.join(
        get_package_share_directory('holohover_utils'), 
        'config', 
        'experiments', 
        experiment_filename
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
    hovercrafts = data["hovercrafts"]
     
    hovercraft_names = []
    hovercraft_ids = []
    initial_states = {'x': [], 'y': [], 'theta': [], 'vx': [], 'vy': [], 'w': []}
    holohover_params = []
    number_of_hovercrafts = data['experiment']['number_of_hovercrafts']


    for hovercraft in hovercrafts:
        hovercraft_names.append(hovercraft['name'])
        hovercraft_ids.append(int(hovercraft['id']))
        initial_state = hovercraft['initial_state']
        for i, val in enumerate(initial_state):
            initial_states[list(initial_states.keys())[i]].append(float(val))
        holohover_params.append(os.path.join(
            get_package_share_directory('holohover_utils'),
            'config',
            hovercraft['holohover_props']))

    print(f" - - - - STARTING SIMULATION EXPERIMENT  - - - - ")
    print(f"Running experiment:\t\t{data['experiment']['name']}")
    print(f"Experiment description:\t\t{data['experiment']['description']}")

    # Start common nodes
    simulation_env_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(this_dir, 'common/simulation_env.launch.py'))
    )

    simulator_config = os.path.join(
        get_package_share_directory('holohover_utils'),
        'config/common',
        'simulation_config.yaml'
    )

    simulator_node = Node(
        package="holohover_utils",
        executable="simulator",
        parameters=[simulator_config,
                    { "hovercrafts" : hovercraft_ids, 
                      "initial_state_x":       initial_states['x'], 
                      "initial_state_y":       initial_states['y'], 
                      "initial_state_theta":   initial_states['theta'], 
                      "initial_state_vx":      initial_states['vx'], 
                      "initial_state_vy":      initial_states['vy'], 
                      "initial_state_w":       initial_states['w'],
                      "holohover_props_files": holohover_params                                          
                    }],
        output='screen'
    )

    ld.add_action(simulator_node)
    ld.add_action(simulation_env_launch)
   
    # Now iterate on each hovercraft and launch the nodes for each one
    print(f"Starting {number_of_hovercrafts} hovercrafts")
    for i in range(number_of_hovercrafts):
        print(f"\t- hovercraft\t\tID: {hovercraft_ids[i]} - Name: {hovercraft_names[i]}")
        namespace = hovercraft_names[i]
        print(f"Namespace: {namespace}")
        print(f"Configuration file: {holohover_params[i]}")
        
        specific_configuration = os.path.join(
            get_package_share_directory('holohover_utils'),
            'config/hovercrafts',
            namespace,
            'config.yaml'
        )

        navigation_node = Node(
            package="holohover_gnc",
            executable="navigation",
            parameters=[navigation_config, {'holohover_props_file' : holohover_params[i]}],
            namespace= namespace,
            output='screen'
        )
        
        controller_node = Node(
            package="holohover_gnc",
            executable="control_lqr",
            parameters=[control_lqr_config, {'holohover_props_file' : holohover_params[i]}],
            name = "control_lqr",
            namespace= namespace,
            output='screen'
        )
        
        rviz_interface_node = Node(
            package="holohover_utils",
            executable="rviz_interface",
            parameters=[specific_configuration, {'id': hovercraft_ids[i], 'holohover_props_file' : holohover_params[i]}],
            namespace= namespace,
            output='screen'
        )
        
        ld.add_action(controller_node)        
        ld.add_action(navigation_node)
        ld.add_action(rviz_interface_node)
    return ld
