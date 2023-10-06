from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

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

    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-s', 'mcap'],
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
