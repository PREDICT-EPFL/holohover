import os
import sys
import pytest
import unittest

import launch
import launch_ros.actions
from ament_index_python import get_package_share_directory
import launch_testing.actions
import launch_testing.markers


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
  ld = launch.LaunchDescription([
    launch.actions.DeclareLaunchArgument(
      name='ft_sensor_a_name',
      default_value='ft_sensor0'
    ),
    launch_ros.actions.Node(
      package='rokubimini_ethercat',
      executable='rokubimini_ethercat_bus_manager_node',
      name='bus0',
      output='screen',
      on_exit=launch.actions.Shutdown(),
      parameters=[
        os.path.join(get_package_share_directory('bota_driver_testing'), 'config', 'rokubimini_sensor_test_ft_offsets.yaml'),
        {
          'time_step': 0.01
        },
        {
          'num_spinners': 2
        },
        {
          'ethercat_bus': 'enx3c8cf8fb1b86'
        },
        {
          'rokubiminis': ['ft_sensor0']  # ['ft_sensor0', 'ft_sensor1']
        },
        {
          'rokubiminis/ft_sensor0/name': launch.substitutions.LaunchConfiguration('ft_sensor_a_name')
        },
        {
          'rokubiminis/ft_sensor0/product_name': 'BFT-ROKA-ECAT-M8'
        },
        {
          'rokubiminis/ft_sensor0/ethercat_address': 1
        }
      ]
    ),
    launch_ros.actions.Node(
      package='bota_driver_testing',
      executable='bota_driver_testing_ft_offsets',
      name='ft_offsets',
      output='screen',
      on_exit=launch.actions.Shutdown(),
      parameters=[
        {
          'topic_name': '/bus0/ft_sensor0/ft_sensor_readings/wrench'
        },
        {
          'test_duration': 30
        },
      ]
    ),
    launch_testing.actions.ReadyToTest()
  ])
  return ld


class TestBotaDriverResetService(unittest.TestCase):
  def test_gtest_run_complete(self, proc_info):
    proc_info.assertWaitForShutdown("bota_driver_testing_ft_offsets", timeout=100.0)

@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

  def test_exit_codes(self, proc_info):
    """Check if the processes exited normally."""
    launch_testing.asserts.assertExitCodes(proc_info)
