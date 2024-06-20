
import os
import launch.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    ld = LaunchDescription()
    this_dir = os.path.dirname(os.path.abspath(__file__))


    params = '/root/ros2_ws/src/holohover/holohover_utils/config/common/holohover_params.yaml'
    
    simulator_config = os.path.join(
        get_package_share_directory('holohover_utils'),
        'config/common',
        'simulation_config.yaml'
    )

    recorder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(this_dir, 'common', 'recorder.launch.py'))
    )


    rviz_interface_node = Node(
        package="holohover_utils",
        executable="rviz_interface",
        parameters=[simulator_config,
                    { "hovercraft_ids" :       [0], 
                      "hovercraft_names" :     ['h0'],
                      "initial_state_x":       [0.0], 
                      "initial_state_y":       [0.0], 
                      "initial_state_theta":   [0.0], 
                      "initial_state_vx":      [0.0], 
                      "initial_state_vy":      [0.0],
                      "initial_state_w":       [0.0],
                      "holohover_props_files": [params],
                      "rviz_props_file": params,
                      "color": [0.0, 0.0, 1.0, 1.0],                                        
                    }],
        output='screen'
    )

    optitrack_node = Node(
        package="holohover_utils",
        executable="optitrack_interface",
        parameters=[simulator_config,
                    { "hovercraft_ids" :       [0], 
                      "hovercraft_names" :     ['h0'],
                      "initial_state_x":       [0.0], 
                      "initial_state_y":       [0.0], 
                      "initial_state_theta":   [0.0], 
                      "initial_state_vx":      [0.0], 
                      "initial_state_vy":      [0.0],
                      "initial_state_w":       [0.0],
                      "holohover_props_files": [params]
                    }],
        output='screen'
    )
    

    ld.add_action(optitrack_node)
    ld.add_action(rviz_interface_node)
    print("including recorder!")
    ld.add_action(recorder_launch)

    return ld


    