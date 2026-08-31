#!/usr/bin/env python3
# Copyright (c) 2026 GHand ROS2 Maintainers
# SPDX-License-Identifier: Apache-2.0

"""Launch two prefixed GHands in Gazebo with MoveIt support."""

import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
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
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    world = LaunchConfiguration("world")
    limit_tolerance = LaunchConfiguration("limit_tolerance")

    declared_args = [
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

    description_share_parent = os.path.dirname(
        get_package_share_directory("ghand_description")
    )
    ros_lib = os.path.join(get_package_prefix("gz_ros2_control"), "lib")
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
    env_actions = [
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH", description_share_parent
        ),
        AppendEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", ros_lib),
    ]
    for library_path in (ros_lib_multiarch, rviz_ogre_lib):
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
            " --limit-tolerance ",
            limit_tolerance,
        ],
        on_stderr="warn",
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

    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", world],
        output="screen",
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
    )
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )
    clock_bridge_starter = RegisterEventHandler(
        OnProcessStart(target_action=gazebo, on_start=[clock_bridge])
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
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
            output="both",
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
        + [
            SetParameter(name="use_sim_time", value=True),
            gazebo,
            clock_bridge_starter,
            rsp,
            spawn_entity,
        ]
        + [jsb_spawner, left_after_jsb, right_after_left]
        + [move_group, rviz]
    )
