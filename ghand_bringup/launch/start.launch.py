#!/usr/bin/env python3
"""Public scene entry: planning preview, Gazebo, or physical GHand control."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    backend, hand = value("backend"), value("hand")
    planning = value("planning") == "true"
    common = {"product_name": value("product_name"), "use_rviz": value("use_rviz")}

    def include(package, filename, arguments):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory(package)) / "launch" / filename
        )), launch_arguments=arguments.items())

    if backend == "preview":
        if not planning:
            raise RuntimeError("backend:=preview requires planning:=true")
        return [include("ghand_moveit_config", "moveit.launch.py", {
            **common, "hand_side": hand, "mode": "preview",
        })]
    if backend == "gazebo":
        arguments = {**common, "planning": value("planning")}
        if hand != "dual":
            arguments["hand_side"] = hand
        if value("world"):
            arguments["world"] = value("world")
        return [include("ghand_gazebo", "dual_hand_gazebo.launch.py" if hand == "dual"
                        else "ghand_gazebo.launch.py", arguments)]

    for package in ("ghand_hardware_interface", "ghand_msgs", "ghand_controllers"):
        try:
            get_package_share_directory(package)
        except LookupError as error:
            raise RuntimeError(
                f"Physical control requires {package}. Build the workspace with "
                "build.sh and source install/setup.bash."
            ) from error

    arguments = {**common, "use_rviz": "false" if planning else value("use_rviz")}
    if hand == "dual":
        for name in ("left_profile", "right_profile", "left_device", "right_device",
                     "left_ifname", "right_ifname"):
            arguments[name] = value(name)
        filename = "dual_hand.launch.py"
    else:
        arguments.update(hand_side=hand, profile=value("profile") or f"rs485_{hand}",
                         device=value("device"), ifname=value("ifname"))
        filename = "ghand.launch.py"
    actions = [include("ghand_bringup", filename, arguments)]
    if planning:
        actions.append(include("ghand_moveit_config", "moveit.launch.py", {
            **common, "hand_side": hand, "mode": "attached",
        }))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("product_name", default_value="ghand5", choices=sorted(
            path.parent.name for path in
            (Path(get_package_share_directory("ghand_bringup")) / "config").glob("*/controllers.yaml")
        )),
        DeclareLaunchArgument("backend", default_value="gazebo", choices=["preview", "gazebo", "hardware"]),
        DeclareLaunchArgument("hand", default_value="right", choices=["left", "right", "dual"]),
        DeclareLaunchArgument("planning", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("use_rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("profile", default_value="", description="Hardware only; defaults to rs485_<hand>."),
        DeclareLaunchArgument("device", default_value=""),
        DeclareLaunchArgument("ifname", default_value=""),
        DeclareLaunchArgument("left_profile", default_value="rs485_left"),
        DeclareLaunchArgument("right_profile", default_value="rs485_dual_right"),
        DeclareLaunchArgument("left_device", default_value=""),
        DeclareLaunchArgument("right_device", default_value=""),
        DeclareLaunchArgument("left_ifname", default_value=""),
        DeclareLaunchArgument("right_ifname", default_value=""),
        DeclareLaunchArgument("world", default_value="", description="Gazebo world override."),
        OpaqueFunction(function=launch_setup),
    ])
