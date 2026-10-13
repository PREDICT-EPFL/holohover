from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from ament_index_python.packages import get_package_share_directory
import datetime
import os


def launch_setup(context):
    ld = []

    name   = LaunchConfiguration('name').perform(context)

    record_arg = DeclareLaunchArgument(
        'record', default_value='false',
        choices=['true', 'false'],
        description='Start ros bag recording of all topics'
    )

    filename = os.path.join(
        get_package_share_directory('holohover_utils'),
        '../../../../log/',
        datetime.datetime.now().strftime('rosbag-' + name + '-%Y-%m-%d_%H-%M-%S')
    )

    print("ROSBAG logging in: ", filename)

    recorder = ExecuteProcess(
        name=name+"_recorder",
        cmd=['ros2', 'bag', 'record', '-s', 'mcap', '-o', filename, '--regex',"/" + name + "/"],
        output='screen',
        condition=IfCondition(LaunchConfiguration('record'))
    )

    

    ld.append(record_arg)
    ld.append(recorder)

    return ld


def generate_launch_description():
    ld = LaunchDescription()

    opfunc = OpaqueFunction(function = launch_setup)

    ld.add_action(DeclareLaunchArgument(
        'name', default_value='h0',
        description='Hovercraft name.'
    ))

    ld.add_action(opfunc)   

    return ld