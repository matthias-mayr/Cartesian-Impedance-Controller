# Copyright (c) 2024, Matthias Mayr
#
# SPDX-License-Identifier: BSD-3-Clause
#
# Minimal ROS 2 simulation launch file for the CartesianImpedanceController.
#
# Replaces the ROS 1 res/launch/examples.launch.  Uses mock_components/GenericSystem
# as a headless hardware backend (no Gazebo required) so the integration tests can
# run in any CI / dev-container environment.
#
# Nodes started:
#   1. robot_state_publisher  — publishes TF from the URDF
#   2. ros2_control_node      — controller manager with mock hardware
#   3. spawner (JSB)          — joint_state_broadcaster
#   4. spawner (CIC)          — CartesianImpedance_trajectory_controller
#
# Usage:
#   ros2 launch cartesian_impedance_controller minimal_mock_simulation.launch.py
#
# The launch file is also included automatically by the integration-test launch
# files (ros_tests.launch.py, ros_func_tests.launch.py) when running via colcon.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("cartesian_impedance_controller")

    # -------------------------------------------------------------------------
    # Paths to configuration files installed into share/
    # -------------------------------------------------------------------------
    urdf_file = os.path.join(pkg_share, "res", "urdf", "robot.urdf.xacro")
    controllers_yaml = os.path.join(pkg_share, "res", "config", "example_controller.yaml")

    # -------------------------------------------------------------------------
    # Process URDF with xacro — wrap in ParameterValue so launch passes it
    # as a plain string, not YAML.
    # -------------------------------------------------------------------------
    robot_description_content = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", urdf_file]),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    # -------------------------------------------------------------------------
    # robot_state_publisher
    # -------------------------------------------------------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # -------------------------------------------------------------------------
    # ros2_control_node  (controller manager + mock hardware)
    # Does NOT receive robot_description directly — it subscribes to the
    # /robot_description topic published by robot_state_publisher.  Passing
    # robot_description as a node parameter would cause controller_manager to
    # forward the full XML to spawned controllers, overriding their own
    # robot_description parameter (which should be the parameter-name string).
    # -------------------------------------------------------------------------
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_yaml],
        output="screen",
    )

    # -------------------------------------------------------------------------
    # Spawners — launched after the controller manager is ready
    # -------------------------------------------------------------------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    cartesian_impedance_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "CartesianImpedance_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # Start JSB spawner once the controller manager node is running, then wait
    # 2 s before starting the main controller spawner so JSB is fully active.
    jsb_after_cm = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner]),
            ],
        )
    )

    cic_after_jsb = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[
                TimerAction(period=2.0, actions=[cartesian_impedance_controller_spawner]),
            ],
        )
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            ros2_control_node,
            jsb_after_cm,
            cic_after_jsb,
        ]
    )
