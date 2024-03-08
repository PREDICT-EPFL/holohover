from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    print("eddai")
    

    ld.add_action(DeclareLaunchArgument(
        'namespace',
        default_value='common',
        description='Namespace of this hovercraft'))
    
    namespace = LaunchConfiguration('namespace')
    
    print(namespace)

    controller_node = Node(
        package="holohover_gnc",
        executable="control_lqr",
        #parameters=[holohover_params, control_lqr_config],
        #name = (namespace + "/control_lqr"),
        namespace=namespace,
        output='screen'
    )
    ld.add_action(controller_node)

  
    return ld
