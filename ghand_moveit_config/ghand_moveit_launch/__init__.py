"""Shared planning nodes for preview, mock, simulation, and physical hardware."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch.actions import AppendEnvironmentVariable, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def planning_nodes(product, side, description, use_rviz, *, use_sim_time=False, execute=True):
    prefix = "dual_" if side == "dual" else ""
    share = Path(get_package_share_directory("ghand_moveit_config"))
    config = share / "config" / product
    ompl = load_yaml(share / "config" / "ompl_planning.yaml")
    if "planner_configs" not in ompl:
        defaults = Path(get_package_share_directory("moveit_configs_utils")) / "default_configs"
        ompl.update(load_yaml(defaults / "ompl_defaults.yaml"))
    parameters = {
        **description,
        "robot_description_semantic": (config / f"{product}_{side}.srdf").read_text(encoding="utf-8"),
        "robot_description_planning": load_yaml(config / f"{prefix}joint_limits.yaml"),
        "robot_description_kinematics": load_yaml(config / f"{prefix}kinematics.yaml"),
        **load_yaml(config / f"{prefix}moveit_controllers.yaml"),
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl,
        "use_sim_time": use_sim_time,
        "allow_trajectory_execution": execute,
        "moveit_manage_controllers": False,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description_semantic": True,
        # Humble's world monitor starts Octomap unconditionally. An empty
        # plugin entry explicitly disables sensor updates on this hand.
        "octomap_resolution": 0.1,
        "sensors": ["disabled"],
        "disabled.sensor_plugin": "",
    }

    log_args = ["--log-level", LaunchConfiguration("log_level", default="info")]
    nodes = [
        Node(package="moveit_ros_move_group", executable="move_group",
             parameters=[parameters], ros_arguments=log_args, output="screen"),
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", str(config / f"{prefix}moveit.rviz")],
             parameters=[parameters], ros_arguments=log_args,
             condition=IfCondition(use_rviz), output="screen"),
    ]
    library_path = Path(get_package_prefix("moveit_ros_move_group")) / "lib/x86_64-linux-gnu"
    if library_path.is_dir():
        return [GroupAction([
            AppendEnvironmentVariable("LD_LIBRARY_PATH", str(library_path)), *nodes,
        ])]
    return nodes
