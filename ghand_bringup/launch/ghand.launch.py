#!/usr/bin/env python3
# Copyright (c) 2026 GHand ROS2 Maintainers
# SPDX-License-Identifier: Apache-2.0

"""Main launch file for the GHand ros2_control stack.

Launches the robot state publisher, controller manager, and default controllers
for a real GHand communication backend. The backend is
selected via the `profile` argument (a YAML file under
ghand_bringup/config/profiles), while `hand_side` selects the left-
or right-hand description.
"""

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
    product_arg = DeclareLaunchArgument(
        "product_name", default_value="ghand5",
        choices=sorted(
            path.parent.name
            for path in (
                Path(get_package_share_directory("ghand_bringup")) / "config"
            ).glob("*/controllers.yaml")
        ),
        description="Product configuration directory (default: ghand5).",
    )
    hand_side_arg = DeclareLaunchArgument(
        "hand_side",
        default_value="right",
        choices=["left", "right"],
        description="Hand model to launch.",
    )
    hand_side = LaunchConfiguration("hand_side")

    # Launch argument: select a communication backend profile
    profile_arg = DeclareLaunchArgument(
        "profile",
        default_value="ethercat_default",
        description=(
            "Communication profile under "
            "ghand_bringup/config/profiles/."
        ),
    )

    # Legacy launch arguments (empty defaults mean "use profile value").
    # When provided, they override the corresponding profile field.
    communication_type_arg = DeclareLaunchArgument(
        "communication_type",
        default_value="",
        description=(
            "Override: communication backend "
            "(ethercat, rs485, canfd; zqwl_canfd is a compatibility alias)."
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
    mock = LaunchConfiguration("mock")
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ghand_bringup"),
                    "config",
                    "ghand_hardware_description.urdf.xacro",
                ]
            ),
            " hand_side:=",
            hand_side,
            " mock:=",
            mock,
            " product_name:=",
            product_name,
            " profile_file:=",
            PathJoinSubstitution(
                [
                    FindPackageShare("ghand_bringup"),
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
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        ros_arguments=ros_log_arguments(),
        output="screen",
        parameters=[robot_description],
    )

    controller_params = PathJoinSubstitution([
        FindPackageShare("ghand_bringup"), "config", product_name, "controllers.yaml",
    ])

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        ros_arguments=ros_log_arguments(),
        on_exit=[EmitEvent(event=Shutdown(reason="GHand controller manager exited"))],
        parameters=[
            {"hardware_components_initial_state": {"unconfigured": ["ghand_hardware"]}},
            PathJoinSubstitution(
                [
                    FindPackageShare("ghand_bringup"),
                    "config",
                    product_name,
                    "ros2_control.yaml",
                ]
            ),
            controller_params,
        ],
        output="screen",
    )

    def setup(context):
        stages = [
            ("activate ghand_hardware", wait_for_control("hardware", "ghand_hardware")),
            *trajectory_stages("joint_trajectory_controller", params_file=controller_params),
        ]
        ready = []
        if mock.perform(context) != "true":
            ready.extend(controller_spawner(name, params_file=controller_params) for name in ["joint_status_broadcaster", "tactile_broadcaster"])
            ready.extend(
                controller_spawner(name, "--inactive", params_file=controller_params)
                for name in ("position_controller", "velocity_percent_controller",
                             "current_percent_controller")
            )
        if LaunchConfiguration("planning").perform(context) == "true":
            ready.extend(planning_after_start(
                product_name.perform(context), LaunchConfiguration("hand_side").perform(context), robot_description,
                use_rviz,
            ))
        else:
            ready.append(Node(
                package="rviz2", executable="rviz2",
                arguments=["-d", PathJoinSubstitution([
                    FindPackageShare("ghand_description"), "config", product_name, "ghand.rviz",
                ])],
                condition=IfCondition(use_rviz), ros_arguments=ros_log_arguments(), output="screen",
            ))
        return start_in_order(stages, ready)

    return LaunchDescription([
        product_arg,
        hand_side_arg,
        profile_arg,
        DeclareLaunchArgument("use_rviz", default_value="true"),
        communication_type_arg,
        device_arg,
        baud_rate_arg,
        dbit_rate_arg,
        slave_id_arg,
        ifname_arg,
        DeclareLaunchArgument("mock", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("planning", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=setup),
        robot_state_pub_node,
        controller_manager_node,
    ])
