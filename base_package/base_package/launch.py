from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    simulator_node = Node(
        package="base_package",
        executable="simulator",
    )

    controller_node = Node(
        package="base_package",
        executable="controller"
    )

    estimator_node = Node(
        package="base_package",
        executable="estimator"
    )
    
    rqt_visualisation = Node(
    	package = "rqt_gui",
    	respawn = "false", 
    	output = "screen",
    	executable = "rqt_gui",
    	args = "--perspective-fila $(find my_pkg_with_rqt_config)/rqt_config/controller.perspective")
    	
    	
    ld.add_action(simulator_node)
    ld.add_action(estimator_node)
    #ld.add_action(controller_node)
    return ld
