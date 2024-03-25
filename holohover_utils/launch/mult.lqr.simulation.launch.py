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

    print(f"Launching experiment from file: {experiment_filename}")

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
    hovercrafts = data["hovercrafts"]
     
    hovercraft_names = []
    hovercraft_ids = []
    initial_states = {'x': [], 'y': [], 'theta': [], 'vx': [], 'vy': [], 'w': []}
    holohover_params = []

    for hovercraft in hovercrafts:
        hovercraft_names.append(hovercraft['name'])
        hovercraft_ids.append(int(hovercraft['id']))
        holohover_params.append(hovercraft['holohover_props'])

        initial_state = hovercraft['initial_state']
        for i, val in enumerate(initial_state):
            initial_states[list(initial_states.keys())[i]].append(float(val))
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

    recorder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(this_dir, 'recorder.launch.py'))
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(this_dir, 'rviz.launch.py'))
    )


    launch_description.append(rviz_launch)
    launch_description.append(recorder_launch)
    launch_description.append(simulator_node)
    #################### COMMON NODES STARTING - END ####################
   
    #################### HOVERCRAFTS STARTING ####################
    # Now iterate on each hovercraft and launch the nodes for each one
    print(f"Starting {len(hovercrafts)} hovercrafts")
    for i in range(len(hovercrafts)):
        hovercraft_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(this_dir, 'hovercraft.launch.py')),
            launch_arguments={'index': str(i), 'name': hovercraft_names[i], 'params': holohover_params[i]}.items()
        )
        
        launch_description.append(hovercraft_launch)
    #################### HOVERCRAFTS STARTING - END ####################

    return launch_description


def generate_launch_description():
    ld = LaunchDescription()

    opfunc = OpaqueFunction(function = launch_setup)

    ld.add_action(DeclareLaunchArgument(
        'experiment', default_value='experiment1.yaml',
        description='Experiment File'
    ))

    ld.add_action(opfunc)   

    return ld

