#!/usr/bin/env python3
# Copyright (c) 2026 GHand ROS2 Maintainers
# SPDX-License-Identifier: Apache-2.0

"""Main launch file for the GHand ros2_control stack.

Launches the robot state publisher, controller manager, and default controllers
for either mock hardware or a real GHand communication backend. The backend is
selected via the `profile` argument (a YAML file under
ghand_hardware_interface/config/profiles), while `hand_side` selects the left-
or right-hand description.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.conditions import UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hand_side_arg = DeclareLaunchArgument(
        "hand_side",
        default_value="right",
        choices=["left", "right"],
        description="Hand model to launch.",
    )
    hand_side = LaunchConfiguration("hand_side")

    # Launch argument: use mock hardware
    use_mock_hardware_arg = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="false",
        description="Use mock hardware instead of real GHand hardware.",
    )
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    # Launch argument: select a communication backend profile
    profile_arg = DeclareLaunchArgument(
        "profile",
        default_value="ethercat_default",
        description=(
            "Communication profile under "
            "ghand_hardware_interface/config/profiles/."
        ),
    )

    # Legacy launch arguments (empty defaults mean "use profile value").
    # When provided, they override the corresponding profile field.
    communication_type_arg = DeclareLaunchArgument(
        "communication_type",
        default_value="",
        description=(
            "Override: communication backend "
            "(ethercat, rs485, canfd, zqwl_canfd)."
        ),
    )
    communication_type = LaunchConfiguration("communication_type")

    device_arg = DeclareLaunchArgument(
        "device",
        default_value="",
        description="Override: Serial/CAN device path.",
    )
    device = LaunchConfiguration("device")

    baud_rate_arg = DeclareLaunchArgument(
        "baud_rate",
        default_value="",
        description=(
            "Override: Baud rate for rs485 or CANFD arbitration phase."
        ),
    )
    baud_rate = LaunchConfiguration("baud_rate")

    dbit_rate_arg = DeclareLaunchArgument(
        "dbit_rate",
        default_value="",
        description=(
            "Override: CANFD data phase bit rate (legacy name for data_rate)."
        ),
    )
    dbit_rate = LaunchConfiguration("dbit_rate")

    slave_id_arg = DeclareLaunchArgument(
        "slave_id",
        default_value="",
        description=(
            "Override: Modbus slave ID / CAN device ID "
            "(legacy name for node_id)."
        ),
    )
    slave_id = LaunchConfiguration("slave_id")

    ifname_arg = DeclareLaunchArgument(
        "ifname",
        default_value="",
        description="Override: EtherCAT network interface name.",
    )
    ifname = LaunchConfiguration("ifname")

    # Get URDF via xacro. The xacro loads the selected profile and applies any
    # explicitly provided legacy overrides.
    robot_description_content = Command(
        [
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
            use_mock_hardware,
            " profile_file:=",
            PathJoinSubstitution(
                [
                    FindPackageShare("ghand_hardware_interface"),
                    "config",
                    "profiles",
                    [LaunchConfiguration("profile"), ".yaml"],
                ]
            ),
            " communication_type:=",
            communication_type,
            " device:=",
            device,
            " baud_rate:=",
            baud_rate,
            " dbit_rate:=",
            dbit_rate,
            " slave_id:=",
            slave_id,
            " ifname:=",
            ifname,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
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

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    joint_status_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_status_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        condition=UnlessCondition(use_mock_hardware),
    )

    tactile_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "tactile_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        condition=UnlessCondition(use_mock_hardware),
    )

    joint_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "position_controller",
            "--controller-manager",
            "/controller_manager",
            "--inactive",
        ],
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "velocity_controller",
            "--controller-manager",
            "/controller_manager",
            "--inactive",
        ],
    )

    effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "effort_controller",
            "--controller-manager",
            "/controller_manager",
            "--inactive",
        ],
    )

    rviz_process = ExecuteProcess(
        cmd=[
            "rviz2",
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare("ghand_hardware_interface"),
                    "config",
                    "ghand.rviz",
                ]
            ),
        ],
        output="both",
    )

    nodes = [
        hand_side_arg,
        use_mock_hardware_arg,
        profile_arg,
        communication_type_arg,
        device_arg,
        baud_rate_arg,
        dbit_rate_arg,
        slave_id_arg,
        ifname_arg,
        robot_state_pub_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        joint_status_broadcaster_spawner,
        tactile_broadcaster_spawner,
        joint_traj_controller_spawner,
        position_controller_spawner,
        velocity_controller_spawner,
        effort_controller_spawner,
        rviz_process,
    ]

    return LaunchDescription(nodes)
