#!/usr/bin/env python3
# Copyright (c) 2026 GHand ROS2 Maintainers
# SPDX-License-Identifier: Apache-2.0

"""Launch dual GHand ros2_control, MoveIt, and RViz."""

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
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim")
    use_rviz = LaunchConfiguration("use_rviz")

    declared_args = [
        DeclareLaunchArgument("use_sim", default_value="true"),
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

    ros_lib_multiarch = os.path.join(
        get_package_prefix("moveit_ros_move_group"),
        "lib",
        "x86_64-linux-gnu",
    )
    rviz_ogre_lib = os.path.join(
        get_package_prefix("rviz_ogre_vendor"),
        "opt",
        "rviz_ogre_vendor",
        "lib",
    )
    env_actions = []
    for library_path in (ros_lib_multiarch, rviz_ogre_lib):
        if os.path.isdir(library_path):
            env_actions.append(
                AppendEnvironmentVariable("LD_LIBRARY_PATH", library_path)
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
            PythonExpression(
                ["'true' if '", use_sim, "' == 'true' else 'false'"]
            ),
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

    moveit_config = (
        MoveItConfigsBuilder("ghand5-dual", package_name="ghand_moveit_config")
        .robot_description_semantic(file_path="config/ghand5_dual.srdf")
        .trajectory_execution(file_path="config/dual_moveit_controllers.yaml")
        .joint_limits(file_path="config/dual_joint_limits.yaml")
        .robot_description_kinematics(file_path="config/dual_kinematics.yaml")
        .planning_pipelines(
            default_planning_pipeline="ompl", pipelines=["ompl"]
        )
        .to_moveit_configs()
    )
    moveit_config.robot_description = robot_description

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

    controller_nodes = [
        spawner("joint_state_broadcaster"),
        spawner("left_joint_trajectory_controller"),
        spawner("right_joint_trajectory_controller"),
        spawner("left_joint_status_broadcaster", UnlessCondition(use_sim)),
        spawner("right_joint_status_broadcaster", UnlessCondition(use_sim)),
        spawner("left_tactile_broadcaster", UnlessCondition(use_sim)),
        spawner("right_tactile_broadcaster", UnlessCondition(use_sim)),
    ]

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[moveit_config.to_dict()],
        output="screen",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare("ghand_moveit_config"),
                    "config",
                    "dual_moveit.rviz",
                ]
            ),
        ],
        parameters=[moveit_config.to_dict()],
        condition=IfCondition(use_rviz),
        output="both",
    )

    return LaunchDescription(
        declared_args
        + env_actions
        + [rsp, controller_manager]
        + controller_nodes
        + [move_group, rviz]
    )
