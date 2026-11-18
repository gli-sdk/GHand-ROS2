#!/usr/bin/env python3
"""Public scene entry: planning preview, mock, Gazebo, or physical control."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    backend, hand = value("backend"), value("hand")
    planning = value("planning") == "true"
    common = {"product_name": value("product_name"), "use_rviz": value("use_rviz")}
    profile = ""
    if backend == "hardware":
        profile = (f" profiles={value('left_profile')},{value('right_profile')}" if hand == "dual"
                   else f" profile={value('profile') or f'rs485_{hand}'}")
    scene = LogInfo(msg=f"GHand product={common['product_name']} hand={hand} "
                        f"backend={backend} planning={value('planning')}{profile}")

    def include(package, filename, arguments):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory(package)) / "launch" / filename
        )), launch_arguments=arguments.items())

    if backend == "preview":
        if not planning:
            raise RuntimeError("backend:=preview requires planning:=true")
        return [scene, include("ghand_moveit_config", "moveit.launch.py", {
            **common, "hand_side": hand, "mode": "preview",
        })]
    if backend == "gazebo":
        arguments = {**common, "planning": value("planning"), "headless": value("headless")}
        if hand != "dual":
            arguments["hand_side"] = hand
        arguments["world"] = value("world") or str(
            Path(get_package_share_directory("ghand_gazebo")) / "worlds" / "grasp_world.world"
        )
        return [scene, include("ghand_gazebo", "dual_hand_gazebo.launch.py" if hand == "dual"
                        else "ghand_gazebo.launch.py", arguments)]

    packages = ("ghand_hardware_interface", "ghand_msgs", "ghand_controllers") if backend == "hardware" else ()
    for package in packages:
        try:
            get_package_share_directory(package)
        except LookupError as error:
            raise RuntimeError(
                f"Physical control requires {package}. Build the workspace with "
                "build.sh and source install/setup.bash."
            ) from error

    arguments = {**common, "planning": value("planning"),
                 "mock": "true" if backend == "mock" else "false"}
    if hand == "dual":
        for name in ("left_profile", "right_profile", "left_device", "right_device",
                     "left_ifname", "right_ifname"):
            arguments[name] = value(name)
        filename = "dual_hand.launch.py"
    else:
        arguments.update(hand_side=hand, profile=value("profile") or f"rs485_{hand}",
                         device=value("device"), ifname=value("ifname"))
        filename = "ghand.launch.py"
    return [scene, include("ghand_bringup", filename, arguments)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("product_name", default_value="ghand5", choices=sorted(
            path.parent.name for path in
            (Path(get_package_share_directory("ghand_bringup")) / "config").glob("*/controllers.yaml")
        )),
        DeclareLaunchArgument("backend", default_value="gazebo", choices=["preview", "mock", "gazebo", "hardware"]),
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
        DeclareLaunchArgument("headless", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("world", default_value="", description="Gazebo world override."),
        DeclareLaunchArgument("log_level", default_value="info",
                              choices=["debug", "info", "warn", "error", "fatal"]),
        DeclareLaunchArgument("sdk_log_level", default_value="info",
                              choices=["debug", "info", "warn", "error"]),
        SetEnvironmentVariable("GHAND_SDK_LOG_LEVEL", LaunchConfiguration("sdk_log_level")),
        OpaqueFunction(function=launch_setup),
    ])
