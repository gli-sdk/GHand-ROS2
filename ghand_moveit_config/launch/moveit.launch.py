#!/usr/bin/env python3
# Copyright (c) 2026 GHand ROS2 Maintainers
# SPDX-License-Identifier: Apache-2.0

"""GHand MoveIt 2 launch file.

Parameters:
  hand_side: left or right hand model (default: right)
  use_sim:   true=mock hardware, false=real hardware (default: true)
  use_rviz:  whether to launch RViz (default: true)
  profile:   communication profile used when use_sim is false
             (default: ethercat_default)
  ifname:    EtherCAT network interface override (default: "")
"""

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
    hand_side = LaunchConfiguration("hand_side")
    use_sim = LaunchConfiguration("use_sim")
    use_rviz = LaunchConfiguration("use_rviz")
    profile = LaunchConfiguration("profile")
    ifname = LaunchConfiguration("ifname")

    # MoveIt/OMPL and RViz vendor libraries live in ROS-specific library
    # directories which are not always inherited by processes spawned from a
    # launch file. Add them explicitly so a normal ros2 launch invocation works.
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

    declared_args = [
        DeclareLaunchArgument(
            "hand_side",
            default_value="right",
            choices=["left", "right"],
            description="Hand model to launch.",
        ),
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
        " hand_side:=",
        hand_side,
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

    # MoveIt needs a concrete robot/SRDF name at launch construction time.
    # Build one conditioned node pair per side while sharing the dynamically
    # selected robot_description and all side-independent planning settings.
    moveit_nodes = []
    for side in ("left", "right"):
        moveit_config = (
            MoveItConfigsBuilder(
                f"ghand5-{side}", package_name="ghand_moveit_config"
            )
            .robot_description_semantic(
                file_path=f"config/ghand5_{side}.srdf"
            )
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
        side_condition = IfCondition(
            PythonExpression(["'", hand_side, f"' == '{side}'"])
        )
        rviz_condition = IfCondition(
            PythonExpression(
                [
                    "'",
                    use_rviz,
                    "' == 'true' and '",
                    hand_side,
                    f"' == '{side}'",
                ]
            )
        )
        moveit_nodes.extend(
            [
                Node(
                    package="moveit_ros_move_group",
                    executable="move_group",
                    output="screen",
                    parameters=[moveit_config.to_dict()],
                    condition=side_condition,
                ),
                Node(
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
                    condition=rviz_condition,
                ),
            ]
        )

    return LaunchDescription(
        declared_args
        + env_actions
        + [
            rsp,
            ros2_control_node,
            jsb_spawner,
            jstatus_spawner,
            tactile_spawner,
            jtc_spawner,
        ]
        + moveit_nodes
    )
