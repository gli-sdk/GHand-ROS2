#!/usr/bin/env python3
# Copyright (c) 2026 GHand ROS2 Maintainers
# SPDX-License-Identifier: Apache-2.0

"""Launch two prefixed GHands under one ros2_control manager."""

import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_mock = LaunchConfiguration("use_mock_hardware")
    use_rviz = LaunchConfiguration("use_rviz")

    declared_args = [
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description=(
                "Use two mock hardware systems instead of physical hands."
            ),
        ),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("left_profile", default_value="rs485_left"),
        DeclareLaunchArgument(
            "right_profile", default_value="rs485_dual_right"
        ),
        DeclareLaunchArgument("left_device", default_value=""),
        DeclareLaunchArgument("right_device", default_value=""),
        DeclareLaunchArgument("left_ifname", default_value=""),
        DeclareLaunchArgument("right_ifname", default_value=""),
    ]

    rviz_ogre_lib = os.path.join(
        get_package_prefix("rviz_ogre_vendor"),
        "opt",
        "rviz_ogre_vendor",
        "lib",
    )
    env_actions = []
    if os.path.isdir(rviz_ogre_lib):
        env_actions.append(
            AppendEnvironmentVariable("LD_LIBRARY_PATH", rviz_ogre_lib)
        )

    profile_dir = PathJoinSubstitution(
        [FindPackageShare("ghand_bringup"), "config", "profiles"]
    )
    description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ghand_bringup"),
                    "config",
                    "ghand_dual_hardware_description.urdf.xacro",
                ]
            ),
            " use_mock_hardware:=",
            use_mock,
            " left_profile_file:=",
            PathJoinSubstitution(
                [profile_dir, [LaunchConfiguration("left_profile"), ".yaml"]]
            ),
            " right_profile_file:=",
            PathJoinSubstitution(
                [profile_dir, [LaunchConfiguration("right_profile"), ".yaml"]]
            ),
            " left_device:=",
            LaunchConfiguration("left_device"),
            " right_device:=",
            LaunchConfiguration("right_device"),
            " left_ifname:=",
            LaunchConfiguration("left_ifname"),
            " right_ifname:=",
            LaunchConfiguration("right_ifname"),
        ]
    )
    robot_description = {"robot_description": description}

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution(
                [
                    FindPackageShare("ghand_bringup"),
                    "config",
                    "dual_controllers.yaml",
                ]
            ),
        ],
        output="both",
    )

    def spawner(name, condition=None):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager"],
            condition=condition,
            output="both",
        )

    controllers = [
        spawner("joint_state_broadcaster"),
        spawner("left_joint_trajectory_controller"),
        spawner("right_joint_trajectory_controller"),
        spawner(
            "left_joint_status_broadcaster",
            condition=UnlessCondition(use_mock),
        ),
        spawner(
            "right_joint_status_broadcaster",
            condition=UnlessCondition(use_mock),
        ),
        spawner(
            "left_tactile_broadcaster",
            condition=UnlessCondition(use_mock),
        ),
        spawner(
            "right_tactile_broadcaster",
            condition=UnlessCondition(use_mock),
        ),
    ]

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare("ghand_bringup"),
                    "config",
                    "ghand_dual.rviz",
                ]
            ),
        ],
        condition=IfCondition(use_rviz),
        output="both",
    )

    return LaunchDescription(
        declared_args
        + env_actions
        + [rsp, controller_manager]
        + controllers
        + [rviz]
    )
