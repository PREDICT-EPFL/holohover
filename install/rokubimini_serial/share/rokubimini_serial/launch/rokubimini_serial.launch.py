import os
import sys

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
import yaml


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='ft_sensor_a_name',
            default_value='ft_sensor0'
        ),
        launch_ros.actions.Node(
            package='rokubimini_serial',
            executable='rokubimini_serial_bus_manager_node',
            name='bus0',
            output='screen',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                os.path.join(get_package_share_directory('rokubimini'), 'config', 'rokubimini_sensor.yaml'),
                {
                    'num_spinners': 1
                },
                # {
                #     'time_step': 0.01
                # },
                {
                    'port': '/dev/ttyUSB0'
                },
                {
                    'rokubiminis': ['ft_sensor0']  # ['ft_sensor0', 'ft_sensor1']
                },
                {
                    'rokubiminis/ft_sensor0/name': launch.substitutions.LaunchConfiguration(
                        'ft_sensor_a_name')
                },
                {
                    'rokubiminis/ft_sensor0/product_name': 'BFT-MEDS-SER-M8'
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
