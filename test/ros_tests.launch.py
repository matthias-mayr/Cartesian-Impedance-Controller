# Copyright (c) 2024, Matthias Mayr
#
# SPDX-License-Identifier: BSD-3-Clause
#
# Launch test file for ros_tests (ROSBaseTests, ROSParameterTests, ROSReconfigureTests).
# Automatically starts a minimal mock-hardware simulation (no Gazebo required)
# before running the GTest binary.
#
# Usage (manual):
#   ros2 launch cartesian_impedance_controller ros_tests.launch.py
#
# Usage (fully automated via colcon test):
#   colcon test --packages-select cartesian_impedance_controller

import os
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import GTest


@pytest.mark.launch_test
def generate_test_description():
    pkg_share = get_package_share_directory("cartesian_impedance_controller")

    # Start the minimal mock-hardware simulation (ros2_control_node + spawners).
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "res", "launch", "minimal_mock_simulation.launch.py")
        )
    )

    ros_tests = GTest(
        path=os.path.join(
            os.environ.get("AMENT_PREFIX_PATH", "").split(":")[0],
            "lib",
            "cartesian_impedance_controller",
            "ros_tests",
        ),
        output="screen",
    )

    return (
        launch.LaunchDescription(
            [
                sim_launch,
                # Give the simulation time to start before running tests.
                TimerAction(period=10.0, actions=[ros_tests]),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"ros_tests": ros_tests},
    )


class TestROSTestsOutput(unittest.TestCase):
    def test_exit_code(self, proc_info, ros_tests):
        proc_info.assertWaitForShutdown(process=ros_tests, timeout=120)


@launch_testing.post_shutdown_test()
class TestROSTestsAfterShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info, ros_tests):
        launch_testing.asserts.assertExitCodes(proc_info, process=ros_tests)
