#!/usr/bin/env python3
# Copyright (c) 2026 GHand ROS2 Maintainers
# SPDX-License-Identifier: Apache-2.0

"""Launch two prefixed GHands in Gazebo with MoveIt support."""

import os
from pathlib import Path

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.events import Shutdown
from launch.actions import (
    EmitEvent,
    OpaqueFunction,
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    product_name = LaunchConfiguration("product_name").perform(context)
    use_rviz = LaunchConfiguration("use_rviz")
    world = LaunchConfiguration("world")
    limit_tolerance = LaunchConfiguration("limit_tolerance")

    description_share_parent = os.path.dirname(
        get_package_share_directory("ghand_description")
    )
    ros_lib = os.path.join(get_package_prefix("gz_ros2_control"), "lib")
    ros_lib_multiarch = os.path.join(
        get_package_prefix("moveit_ros_move_group"),
        "lib",
        "x86_64-linux-gnu",
    )
    env_actions = [
        AppendEnvironmentVariable(
            "IGN_GAZEBO_RESOURCE_PATH", description_share_parent
        ),
        AppendEnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", ros_lib),
    ]
    for library_path in (ros_lib_multiarch,):
        if os.path.isdir(library_path):
            env_actions.append(
                AppendEnvironmentVariable("LD_LIBRARY_PATH", library_path)
            )

    nvidia_egl = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
    if os.path.exists(nvidia_egl):
        env_actions.extend(
            [
                SetEnvironmentVariable("__NV_PRIME_RENDER_OFFLOAD", "1"),
                SetEnvironmentVariable("__GLX_VENDOR_LIBRARY_NAME", "nvidia"),
                SetEnvironmentVariable(
                    "__EGL_VENDOR_LIBRARY_FILENAMES", nvidia_egl
                ),
            ]
        )

    description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="python3")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ghand_gazebo"),
                    "scripts",
                    "generate_gazebo_robot_description.py",
                ]
            ),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ghand_gazebo"),
                    "config",
                    "ghand_dual_gazebo_description.urdf.xacro",
                ]
            ),
            " --xacro-arg product_name:=",
            product_name,
            " --mimic-config ",
            PathJoinSubstitution([
                FindPackageShare("ghand_gazebo"), "config", product_name, "mimic_joints.yaml"
            ]),
            " --limit-tolerance ",
            limit_tolerance,
        ],
        on_stderr="warn",
    )
    robot_description = {"robot_description": description}

    gazebo = ExecuteProcess(
        on_exit=[EmitEvent(event=Shutdown(reason="Gazebo exited"))],
        cmd=["ign", "gazebo", "-r", world],
        output="screen",
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
    )
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )
    clock_bridge_starter = RegisterEventHandler(
        OnProcessStart(target_action=gazebo, on_start=[clock_bridge])
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "/robot_description",
            "-entity",
            "GHand-dual",
            "-x",
            "0.35",
            "-y",
            "-0.08",
            "-z",
            "0.95",
            "-R",
            "-1.57079632679",
            "-P",
            "3.14159265359",
            "-Y",
            "1.57079632679",
        ],
        output="screen",
    )

    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                name,
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                "30",
                "--switch-timeout",
                "30",
                "--service-call-timeout",
                "30",
            ],
            output="screen",
        )

    jsb_spawner = spawner("joint_state_broadcaster")
    left_spawner = spawner("left_joint_trajectory_controller")
    right_spawner = spawner("right_joint_trajectory_controller")
    left_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[left_spawner])
    )
    right_after_left = RegisterEventHandler(
        OnProcessExit(target_action=left_spawner, on_exit=[right_spawner])
    )
    moveit_nodes = []
    if LaunchConfiguration("planning").perform(context) == "true":
        from ghand_moveit_launch import planning_nodes
        moveit_nodes = planning_nodes(product_name, "dual", robot_description, use_rviz,
                                     use_sim_time=True)

    model_rviz = Node(
        package="rviz2", executable="rviz2",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("ghand_description"), "config", product_name, "ghand_dual.rviz"
        ])],
        parameters=[robot_description],
        condition=IfCondition(use_rviz), output="screen",
    )

    return (
        env_actions
        + [
            SetParameter(name="use_sim_time", value=True),
            gazebo,
            clock_bridge_starter,
            rsp,
            spawn_entity,
        ]
        + [jsb_spawner, left_after_jsb, right_after_left]
        + (moveit_nodes if LaunchConfiguration("planning").perform(context) == "true" else [model_rviz])
    )


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument("planning", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument(
            "product_name", default_value="ghand5",
            choices=sorted(
                path.parent.name
                for path in (
                    Path(get_package_share_directory("ghand_description")) / "urdf"
                ).glob("*/*_right_description.urdf")
            ),
            description="Product configuration directory (default: ghand5).",
        ),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ghand_gazebo"),
                    "worlds",
                    "grasp_world.world",
                ]
            ),
        ),
        DeclareLaunchArgument("limit_tolerance", default_value="-0.01"),
    ]
    return LaunchDescription(
        declared_args + [OpaqueFunction(function=launch_setup)]
    )
