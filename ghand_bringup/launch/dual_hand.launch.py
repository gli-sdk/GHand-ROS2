#!/usr/bin/env python3
# Copyright (c) 2026 GHand ROS2 Maintainers
# SPDX-License-Identifier: Apache-2.0

"""Launch two prefixed GHands under one ros2_control manager."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.events import Shutdown
from launch.actions import OpaqueFunction, DeclareLaunchArgument, EmitEvent
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ghand_bringup_launch import (
    controller_spawner, planning_after_start, ros_log_arguments, start_in_order, trajectory_stages, wait_for_control,
)
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
    mock = LaunchConfiguration("mock")
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
            " mock:=",
            mock,
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
    robot_description = {"robot_description": ParameterValue(description, value_type=str)}

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        ros_arguments=ros_log_arguments(),
        parameters=[robot_description],
        output="screen",
    )
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        ros_arguments=ros_log_arguments(),
        on_exit=[EmitEvent(event=Shutdown(reason="GHand controller manager exited"))],
        remappings=[("~/robot_description", "/robot_description")],
        parameters=[
            {"hardware_components_initial_state": {"unconfigured": ["left_ghand_hardware", "right_ghand_hardware"]}},
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

    def setup(context):
        stages = [
            (f"activate {component}", wait_for_control("hardware", component))
            for component in ("left_ghand_hardware", "right_ghand_hardware")
        ]
        stages.extend(trajectory_stages(
            "left_joint_trajectory_controller", "right_joint_trajectory_controller",
        ))
        ready = []
        if mock.perform(context) != "true":
            ready.extend(controller_spawner(name) for name in [f"{side}_{name}" for side in ("left", "right")
                  for name in ("joint_status_broadcaster", "tactile_broadcaster")])
        if LaunchConfiguration("planning").perform(context) == "true":
            ready.extend(planning_after_start(
                product_name.perform(context), "dual", robot_description,
                use_rviz,
            ))
        else:
            ready.append(Node(
                package="rviz2", executable="rviz2",
                arguments=["-d", PathJoinSubstitution([
                    FindPackageShare("ghand_description"), "config", product_name, "ghand_dual.rviz",
                ])],
                condition=IfCondition(use_rviz), ros_arguments=ros_log_arguments(), output="screen",
            ))
        return start_in_order(stages, ready)

    return LaunchDescription([
        *declared_args,
        DeclareLaunchArgument("mock", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("planning", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=setup),
        rsp,
        controller_manager,
    ])
