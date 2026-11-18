#!/usr/bin/env python3
# Copyright (c) 2026 GHand ROS2 Maintainers
# SPDX-License-Identifier: Apache-2.0
"""Planning preview, or planning attached to an existing physical control stack."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ghand_moveit_launch import planning_nodes


def launch_setup(context):
    product = LaunchConfiguration("product_name").perform(context)
    side = LaunchConfiguration("hand_side").perform(context)
    preview = LaunchConfiguration("mode").perform(context) == "preview"
    share = Path(get_package_share_directory("ghand_moveit_config"))
    description_share = Path(get_package_share_directory("ghand_description"))
    description_path = description_share / f"urdf/{product}/{product}_{side}_description.urdf"
    description = {}
    actions = []
    log_args = ["--log-level", LaunchConfiguration("log_level", default="info")]
    if preview:
        description = {"robot_description": description_path.read_text(encoding="utf-8")}
        initial = yaml.safe_load((share / f"config/{product}/initial_positions.yaml").read_text())["initial_positions"]
        if side == "dual":
            initial = {f"{hand}_{joint}": value for hand in ("left", "right")
                       for joint, value in initial.items()}
        actions.extend([
            Node(package="robot_state_publisher", executable="robot_state_publisher",
                 parameters=[description], ros_arguments=log_args, output="screen"),
            # A fixed kinematic start state; there is no controller or execution backend.
            Node(package="joint_state_publisher", executable="joint_state_publisher",
                 arguments=[str(description_path)],
                 parameters=[description, {"zeros": initial, "rate": 10}],
                 ros_arguments=log_args, output="screen"),
        ])
    actions.extend(planning_nodes(product, side, description, LaunchConfiguration("use_rviz"),
                                  use_sim_time=False, execute=not preview))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("product_name", default_value="ghand5", choices=sorted(
            path.parent.name for path in
            (Path(get_package_share_directory("ghand_description")) / "urdf").glob("*/*_right_description.urdf")
        )),
        DeclareLaunchArgument("hand_side", default_value="right", choices=["left", "right", "dual"]),
        DeclareLaunchArgument("mode", default_value="preview", choices=["preview", "attached"],
                              description="Preview disables execution; attached uses an existing hardware stack."),
        DeclareLaunchArgument("use_rviz", default_value="true", choices=["true", "false"]),
        OpaqueFunction(function=launch_setup),
    ])
