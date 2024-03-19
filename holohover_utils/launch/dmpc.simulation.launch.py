import os
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    ld = LaunchDescription()

    this_dir = os.path.dirname(os.path.abspath(__file__))

    simulation_env_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(this_dir, 'common/simulation_env.launch.py'))
    )

    dmpc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(this_dir, 'common/dmpc.launch.py'))
    )

    # ld.add_action(simulation_env_launch)
    ld.add_action(dmpc_launch)

    return ld