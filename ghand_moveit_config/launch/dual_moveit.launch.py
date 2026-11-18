#!/usr/bin/env python3
"""Dual-hand planning preview, or attachment to an existing hardware stack."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("product_name", default_value="ghand5", choices=sorted(
            path.parent.name for path in
            (Path(get_package_share_directory("ghand_description")) / "urdf").glob("*/*_right_description.urdf")
        )),
        DeclareLaunchArgument("mode", default_value="preview", choices=["preview", "attached"]),
        DeclareLaunchArgument("use_rviz", default_value="true", choices=["true", "false"]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory("ghand_moveit_config")) / "launch/moveit.launch.py"
        )), launch_arguments={
            "hand_side": "dual", "product_name": LaunchConfiguration("product_name"),
            "mode": LaunchConfiguration("mode"), "use_rviz": LaunchConfiguration("use_rviz"),
        }.items()),
    ])
