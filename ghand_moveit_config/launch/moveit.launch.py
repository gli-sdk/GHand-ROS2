#!/usr/bin/env python3
# Copyright (c) 2026 GHand ROS2 Maintainers
# SPDX-License-Identifier: Apache-2.0

"""GHand MoveIt 2 launch file.

Parameters:
  use_sim:   true=mock hardware, false=real hardware (default: true)
  use_rviz:  whether to launch RViz (default: true)
  profile:   communication profile used when use_sim is false
             (default: ethercat_default)
  ifname:    EtherCAT network interface override (default: "")
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    profile = LaunchConfiguration("profile")
    ifname = LaunchConfiguration("ifname")

    declared_args = [
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Use mock hardware when true",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz when true",
        ),
        DeclareLaunchArgument(
            "profile",
            default_value="ethercat_default",
            description=(
                "Communication profile under "
                "ghand_hardware_interface/config/profiles/."
            ),
        ),
        DeclareLaunchArgument(
            "ifname",
            default_value="",
            description="Override: EtherCAT network interface name.",
        ),
    ]

    # URDF via xacro with mock/real switching
    xacro_cmd = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [
                FindPackageShare("ghand_hardware_interface"),
                "config",
                "ghand_hardware_description.urdf.xacro",
            ]
        ),
        " use_mock_hardware:=",
        PythonExpression(["'true' if '", use_sim, "' == 'true' else 'false'"]),
        " profile_file:=",
        PathJoinSubstitution(
            [
                FindPackageShare("ghand_hardware_interface"),
                "config",
                "profiles",
                [profile, ".yaml"],
            ]
        ),
        " ifname:=",
        ifname,
    ]
    robot_description_content = Command(xacro_cmd)

    # Build MoveIt configs (skip robot_description; inject manually after
    # builder). The hand group is a multi-finger tree structure without an IK
    # solver, so we
    # default to and only use the OMPL pipeline.
    moveit_config = (
        MoveItConfigsBuilder("ghand5-right", package_name="ghand_moveit_config")
        .robot_description_semantic(file_path="config/ghand5_right.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl"],
        )
        .to_moveit_configs()
    )
    moveit_config.robot_description = {
        "robot_description": robot_description_content
    }

    # robot_state_publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description_content}],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_content},
            PathJoinSubstitution(
                [
                    FindPackageShare("ghand_hardware_interface"),
                    "config",
                    "ros2_control.yaml",
                ]
            ),
            PathJoinSubstitution(
                [
                    FindPackageShare("ghand_hardware_interface"),
                    "config",
                    "controllers.yaml",
                ]
            ),
        ],
        output="both",
    )

    # Controller spawners
    jtc_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    jstatus_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_status_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        condition=UnlessCondition(use_sim),
    )
    tactile_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "tactile_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        condition=UnlessCondition(use_sim),
    )

    # MoveGroup
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz: pass robot_description_semantic so the MotionPlanning plugin can
    # load SRDF.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare("ghand_moveit_config"),
                    "config",
                    "moveit.rviz",
                ]
            ),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="both",
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        declared_args
        + [
            rsp,
            ros2_control_node,
            jsb_spawner,
            jstatus_spawner,
            tactile_spawner,
            jtc_spawner,
            move_group,
            rviz,
        ]
    )
