#!/usr/bin/env python3
# Copyright (c) 2026 GHand ROS2 Maintainers
# SPDX-License-Identifier: Apache-2.0

"""Launch two prefixed GHands under one ros2_control manager."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.events import Shutdown
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    product_name = LaunchConfiguration("product_name")
    use_rviz = LaunchConfiguration("use_rviz")

    declared_args = [
        DeclareLaunchArgument(
            "product_name", default_value="ghand5",
            choices=sorted(
                path.parent.name
                for path in (
                    Path(get_package_share_directory("ghand_bringup")) / "config"
                ).glob("*/controllers.yaml")
            ),
            description="Product configuration directory (default: ghand5).",
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
            " product_name:=",
            product_name,
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
        output="screen",
    )
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        on_exit=[EmitEvent(event=Shutdown(reason="GHand controller manager exited"))],
        parameters=[
            robot_description,
            PathJoinSubstitution(
                [
                    FindPackageShare("ghand_bringup"),
                    "config",
                    product_name,
                    "dual_controllers.yaml",
                ]
            ),
        ],
        output="screen",
    )

    def spawner(name, condition=None):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager"],
            condition=condition,
            output="screen",
        )

    controllers = [
        spawner("joint_state_broadcaster"),
        spawner("left_joint_trajectory_controller"),
        spawner("right_joint_trajectory_controller"),
        spawner(
            "left_joint_status_broadcaster",
        ),
        spawner(
            "right_joint_status_broadcaster",
        ),
        spawner(
            "left_tactile_broadcaster",
        ),
        spawner(
            "right_tactile_broadcaster",
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
                    product_name,
                    "ghand_dual.rviz",
                ]
            ),
        ],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        declared_args
        + [rsp, controller_manager]
        + controllers
        + [rviz]
    )
