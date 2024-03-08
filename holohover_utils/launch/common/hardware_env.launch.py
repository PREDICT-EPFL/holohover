import os
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    ld = LaunchDescription()

    this_dir = os.path.dirname(os.path.abspath(__file__))

    optitrack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(this_dir, 'optitrack.launch.py'))
    )

    recorder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(this_dir, 'recorder.launch.py'))
    )

    ld.add_action(optitrack_launch)
    ld.add_action(recorder_launch)

    return ld
