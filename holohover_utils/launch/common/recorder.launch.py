from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from ament_index_python.packages import get_package_share_directory
import datetime
import os


def generate_launch_description():
    ld = LaunchDescription()

    record_arg = DeclareLaunchArgument(
        'record', default_value='false',
        choices=['true', 'false'],
        description='Start ros bag recording of all topics'
    )

    record_delay_arg = DeclareLaunchArgument(
        'record_delay', default_value='0.0',
        description='Delay before start recording'
    )

    filename = os.path.join(
        get_package_share_directory('holohover_utils'),
        '../../../../log/',
        datetime.datetime.now().strftime('rosbag-%Y-%m-%d_%H-%M-%S')
    )

    print("ROSBAG logging in: ", filename)

    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-s', 'mcap', '-o', filename, '--regex', '(/h.|/visua|/roso|/trigger)'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('record'))
    )

    recorder_delayed = TimerAction(
        period=LaunchConfiguration('record_delay'),
        actions=[
        	recorder,
        ]
    )

    ld.add_action(record_arg)
    ld.add_action(record_delay_arg)
    ld.add_action(recorder_delayed)

    return ld
