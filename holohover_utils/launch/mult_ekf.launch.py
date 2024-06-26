import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
import yaml

def launch_setup(context):
    
    experiment_filename = LaunchConfiguration('experiment').perform(context)
    machine = LaunchConfiguration('machine').perform(context)

    print(f"Launching experiment from file: {experiment_filename}")
    print(f"This machine is named: {machine}")

    launch_description = []

    this_dir = os.path.dirname(os.path.abspath(__file__))
    
    #################### EXPERIMENT FILE PARSING ####################
    experiment_conf = os.path.join(
        get_package_share_directory('holohover_utils'), 
        'config', 
        'experiments', 
        experiment_filename
    )

    data = yaml.safe_load(open(experiment_conf, 'r'))
    hovercraft = data["hovercraft"]
    obstacles = data["obstacles"] if "obstacles" in data else []
    common_nodes_machine = data["experiment"]["machine"]
    opt_alg = data["experiment"]["opt_alg"] # admm or dsqp

    rviz_props_file = os.path.join(
        get_package_share_directory('holohover_utils'),
        'config',
        data["experiment"]["rviz_props_file"])
     
    hovercraft_machines = []
    hovercraft_names = []
    hovercraft_ids = []
    initial_states = {'x': [], 'y': [], 'theta': [], 'vx': [], 'vy': [], 'w': []}
    holohover_params = []

    hovercraft_names_simulated = []
    hovercraft_ids_simulated = []
    initial_states_simulated = {'x': [], 'y': [], 'theta': [], 'vx': [], 'vy': [], 'w': []}
    holohover_params_simulated = []
    colors = []

    obstacle_machines = []
    obstacle_names = []
    obstacle_params = []
    obstacle_initial_states = {'x': [], 'y': [], 'theta': [], 'vx': [], 'vy': [], 'w': []}

    for h in hovercraft:
        hovercraft_machines.append(h['machine'])
        hovercraft_names.append(h['name'])
        hovercraft_ids.append(int(h['id']))
        colors += h['color']
        holohover_params.append(os.path.join(
            get_package_share_directory('holohover_utils'),
            'config',
            h['holohover_props']))

        initial_state = h['initial_state']
        for i, val in enumerate(initial_state):
            initial_states[list(initial_states.keys())[i]].append(float(val))

        if h['simulate']:
            hovercraft_names_simulated.append(h['name'])
            hovercraft_ids_simulated.append(int(h['id']))
            holohover_params_simulated.append(os.path.join(
                get_package_share_directory('holohover_utils'),
                'config',
                h['holohover_props']))

            initial_state = h['initial_state']
            for i, val in enumerate(initial_state):
                initial_states_simulated[list(initial_states_simulated.keys())[i]].append(float(val))

    for h in obstacles:
        params = os.path.join(
            get_package_share_directory('holohover_utils'),
            'config',
            h['holohover_props'])

        obstacle_machines.append(h['machine'])
        obstacle_names.append(h['name'])
        obstacle_params.append(params)

        hovercraft_machines.append(h['machine'])
        hovercraft_names.append(h['name'])
        hovercraft_ids.append(int(h['id']))
        colors += h['color']
        holohover_params.append(params)

        initial_state = h['initial_state']
        for i, val in enumerate(initial_state):
            initial_states[list(initial_states.keys())[i]].append(float(val))
            obstacle_initial_states[list(obstacle_initial_states.keys())[i]].append(float(val))
            
        if h['simulate']:
            hovercraft_names_simulated.append(h['name'])
            hovercraft_ids_simulated.append(int(h['id']))
            holohover_params_simulated.append(os.path.join(
                get_package_share_directory('holohover_utils'),
                'config',
                h['holohover_props']))

            initial_state = h['initial_state']
            for i, val in enumerate(initial_state):
                initial_states_simulated[list(initial_states_simulated.keys())[i]].append(float(val))

    #################### EXPERIMENT FILE PARSING - END ####################
     
    #################### HOVERCRAFT STARTING ####################
    # Now iterate on each hovercraft and launch the nodes for each one

    navigation_config = os.path.join(
        get_package_share_directory('holohover_utils'),
        'config/common',
        'navigation_config.yaml'
    )
    print(f"Starting {len(hovercraft)} hovercraft EKF")
    for i in range(len(hovercraft)):
        # - - - Nodes
        navigation_node = Node(
            package="holohover_navigation",
            executable="navigation_disturbance",
            parameters=[navigation_config, {'holohover_props_file' : holohover_params[i]}],
            namespace= hovercraft_names[i],
            output='screen',
            #prefix='nice -n -19'
        )
        launch_description.append(navigation_node)


    print(f"Starting {len(obstacles)} obstacles EKF")
    for i in range(len(obstacles)):
        # - - - Nodes
        navigation_node = Node(
            package="holohover_navigation",
            executable="navigation",
            parameters=[navigation_config, {'holohover_props_file' : obstacle_params[i]}],
            namespace= obstacle_names[i],
            output='screen',
            prefix='nice -n -19'
        )
        launch_description.append(navigation_node)
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

