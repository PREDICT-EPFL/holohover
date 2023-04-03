import os

import launch.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    holohover_params = os.path.join(
        get_package_share_directory('holohover_gnc'),
        'config',
        'holohover_params.yaml'
    )

    mocap_config = os.path.join(
        get_package_share_directory('holohover_utils'),
        'config',
        'mocap_config.yaml'
    )
    mocap_node = Node(
        package='mocap_optitrack',
        executable='mocap_node',
        parameters=[mocap_config],
        output='screen'
    )

    optitrack_interface_node = Node(
        package="holohover_utils",
        executable="optitrack_interface",
        output='screen'
    )

    navigation_config = os.path.join(
        get_package_share_directory('holohover_gnc'),
        'config',
        'navigation_config.yaml'
    )
    navigation_node = Node(
        package="holohover_gnc",
        executable="navigation",
        parameters=[holohover_params, navigation_config],
        output='screen'
    )

    control_lqr_config = os.path.join(
        get_package_share_directory('holohover_gnc'),
        'config',
        'control_lqr_config.yaml'
    )
    controller_node = Node(
        package="holohover_gnc",
        executable="control_exp",
        parameters=[holohover_params, control_lqr_config],
        output='screen'
    )

    rviz_interface_node = Node(
        package="holohover_utils",
        executable="rviz_interface",
        parameters=[holohover_params],
        output='screen'
    )

    rviz_config = os.path.join(
        get_package_share_directory('holohover_utils'),
        'gui',
        'holohover_config.rviz'
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    micro_agent = launch.actions.TimerAction(
        period=0.0,
        actions=[
        	Node(
				package="micro_ros_agent",
				executable="micro_ros_agent",
				arguments=["udp4", "-p", "8888"],
				output='screen'
			),
        ]
	)
	
    visualization = launch.actions.TimerAction(
        period=2.0,
        actions=[
        	rviz_node,
        	rviz_interface_node,
        ]
	)
	
    holohover = launch.actions.TimerAction(
        period=6.0,
        actions=[
        	mocap_node,
        	optitrack_interface_node,
        	navigation_node,
        	#controller_node,
        ]
	)

    recorder = launch.actions.TimerAction(
        period=4.0,
        actions=[
        	launch.actions.ExecuteProcess(
				cmd=['ros2', 'bag', 'record', '--all'],
				output='screen'
			),
		]
	)

    
    ld.add_action(micro_agent)  
    ld.add_action(visualization)   
    #ld.add_action(recorder)
    ld.add_action(holohover)
    

    return ld
