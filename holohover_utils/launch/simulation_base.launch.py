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
    machine             = LaunchConfiguration('machine').perform(context)
    print(f"Launching experiment from file: {experiment_filename}")
    print(f"This machine is named: {machine}")

    launch_description  = []
    this_dir            = os.path.dirname(os.path.abspath(__file__))
    
    #################### EXPERIMENT FILE PARSING ####################
    experiment_conf = os.path.join(
        get_package_share_directory('holohover_utils'), 
        'config', 'experiments', experiment_filename
    ) # holohover_utils/config/experiments/experiment_filename

    data        = yaml.safe_load(open(experiment_conf, 'r'))
    hovercraft  = data["hovercraft"]
    obstacles   = data["obstacles"] if "obstacles" in data else []
    common_nodes_machine = data["experiment"]["machine"]
    opt_alg     = data["experiment"]["opt_alg"] # admm or dsqp
    # print(f" Here are {len(hovercraft)} hovercraft/s ")

    #################### rviz configuration ####################
    rviz_props_file = os.path.join(
        get_package_share_directory('holohover_utils'),
        'config',data["experiment"]["rviz_props_file"])
     
    #################### Initialize hovercraft related variables ####
    hovercraft_machines = []
    hovercraft_names    = []
    hovercraft_ids      = []
    initial_states      = {'x': [], 'y': [], 'theta': [], 'vx': [], 'vy': [], 'w': []}
    holohover_params    = []

    hovercraft_names_simulated  = []
    hovercraft_ids_simulated    = []
    initial_states_simulated    = {'x': [], 'y': [], 'theta': [], 'vx': [], 'vy': [], 'w': []}
    holohover_params_simulated  = []
    colors                      = []

    obstacle_machines       = []
    obstacle_names          = []
    obstacle_params         = []
    obstacle_initial_states = {'x': [], 'y': [], 'theta': [], 'vx': [], 'vy': [], 'w': []}

    for h in hovercraft:
        hovercraft_machines.append(h['machine'])
        hovercraft_names.append(h['name'])
        hovercraft_ids.append(int(h['id']))
        initial_state = h['initial_state']
        for i, val in enumerate(initial_state):
            initial_states[list(initial_states.keys())[i]].append(float(val))
        holohover_params.append(os.path.join(
            get_package_share_directory('holohover_utils'),
            'config', 
            h['holohover_props']))

        if h['simulate']:
            hovercraft_names_simulated.append(h['name'])
            hovercraft_ids_simulated.append(int(h['id']))
            initial_state = h['initial_state']
            for i, val in enumerate(initial_state):
                initial_states_simulated[list(initial_states_simulated.keys())[i]].append(float(val))
            holohover_params_simulated.append(os.path.join(
                get_package_share_directory('holohover_utils'),
                'config',
                h['holohover_props']))
        
        colors += h['color']

    for h in obstacles:
        obstacle_machines.append(h['machine'])
        obstacle_names.append(h['name'])
        params = os.path.join(
            get_package_share_directory('holohover_utils'),
            'config',
            h['holohover_props'])
        obstacle_params.append(params)

        hovercraft_machines.append(h['machine'])
        hovercraft_names.append(h['name'])
        hovercraft_ids.append(int(h['id']))
        initial_state = h['initial_state']
        for i, val in enumerate(initial_state):
            initial_states[list(initial_states.keys())[i]].append(float(val))
            obstacle_initial_states[list(obstacle_initial_states.keys())[i]].append(float(val))
        holohover_params.append(params)
            
        if h['simulate']:
            hovercraft_names_simulated.append(h['name'])
            hovercraft_ids_simulated.append(int(h['id']))
            initial_state = h['initial_state']
            for i, val in enumerate(initial_state):
                initial_states_simulated[list(initial_states_simulated.keys())[i]].append(float(val))
            holohover_params_simulated.append(os.path.join(
                get_package_share_directory('holohover_utils'),
                'config',
                h['holohover_props']))
        colors += h['color']

    
    print('initial_states_simulated', initial_states_simulated)
    print('initial_states', initial_states)

    #################### EXPERIMENT FILE PARSING - END ####################
     

    #################### COMMON NODES STARTING ####################
    print(f" - - - - STARTING SIMULATION EXPERIMENT  - - - - ")
    print(f"Running experiment:\t\t{data['experiment']['name']}")
    print(f"Experiment description:\t\t{data['experiment']['description']}")

    simulator_config = os.path.join(
        get_package_share_directory('holohover_utils'),
        'config/common',
        'simulation_config.yaml'
    )

    simulator_node = Node(
        package="holohover_simulator",
        executable="simulator",
        parameters=[simulator_config,
                    { "hovercraft_ids" :       hovercraft_ids_simulated, 
                      "hovercraft_names" :     hovercraft_names_simulated,
                      "initial_state_x":       initial_states_simulated['x'], 
                      "initial_state_y":       initial_states_simulated['y'], 
                      "initial_state_theta":   initial_states_simulated['theta'], 
                      "initial_state_vx":      initial_states_simulated['vx'], 
                      "initial_state_vy":      initial_states_simulated['vy'], 
                      "initial_state_w":       initial_states_simulated['w'],
                      "holohover_props_files": holohover_params_simulated,
                      "are_all_simulated":     True
                    }],
        output='screen'
    )

    rviz_interface_node = Node(
        package="holohover_utils",
        executable="rviz_interface",
        parameters=[simulator_config,
                    { "rviz_props_file" :      rviz_props_file,
                      "hovercraft_ids" :       hovercraft_ids, 
                      "hovercraft_names" :     hovercraft_names,
                      "initial_state_x":       initial_states['x'], 
                      "initial_state_y":       initial_states['y'], 
                      "initial_state_theta":   initial_states['theta'], 
                      "initial_state_vx":      initial_states['vx'], 
                      "initial_state_vy":      initial_states['vy'], 
                      "initial_state_w":       initial_states['w'],
                      "holohover_props_files": holohover_params,
                      "color": colors,                                        
                    }],
        output='screen'
    )

    optitrack_node = Node(
        package="holohover_utils",
        executable="optitrack_interface",
        parameters=[simulator_config,
                    { "hovercraft_ids" :       hovercraft_ids, 
                      "hovercraft_names" :     hovercraft_names,
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

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(this_dir, 'common', 'rviz.launch.py'))
    )

    # ekf_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(this_dir, 'mult_ekf.launch.py')),
    #     launch_arguments=
    #                 {'experiment': experiment_filename}.items()
    # )

    if common_nodes_machine == machine or machine == "all":
        launch_description.append(rviz_interface_node)
        launch_description.append(rviz_launch)
        if len(hovercraft_ids_simulated) != 0:
           launch_description.append(simulator_node)
        launch_description.append(optitrack_node)
        #launch_description.append(ekf_launch)      #????
        
        
    #################### COMMON NODES STARTING - END ####################

    #################### HOVERCRAFT STARTING ####################
    # Now iterate on each hovercraft and launch the nodes for each one
    print(f"Starting {len(hovercraft)} hovercraft")
    for i in range(len(hovercraft)):
        if hovercraft_machines[i] == machine or machine == "all":
            hovercraft_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(this_dir, 'hovercraft_lqr.launch.py')),
                launch_arguments={'index': str(i), 'name': hovercraft_names[i], 'params': holohover_params[i]}.items()
            )
            launch_description.append(hovercraft_launch)
    #################### HOVERCRAFT STARTING - END ####################
   

    return launch_description


def generate_launch_description():
    ld = LaunchDescription()

    opfunc = OpaqueFunction(function = launch_setup)

    ld.add_action(DeclareLaunchArgument(
        'experiment', default_value='simulation_base.yaml',
        description='Experiment File'
    ))

    ld.add_action(DeclareLaunchArgument(
        'machine', default_value='master',
        description='Machine Name'
    ))

    ld.add_action(opfunc)
 
    return ld

