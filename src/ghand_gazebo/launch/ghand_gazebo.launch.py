"""GHand Gazebo simulation launch file.

Parameters:
  use_rviz: 是否启动 RViz (默认: true)
  world:    Gazebo world 文件路径 (默认: empty.world)
"""

import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    world = LaunchConfiguration("world")

    declared_args = [
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Launch RViz when true"),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ghand_gazebo"), "worlds", "empty.world"]
            ),
            description="Gazebo world file path",
        ),
    ]

    # URDF via xacro with Gazebo mode enabled.
    xacro_cmd = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("ghand_description"), "urdf", "G5-A7-R.urdf"]),
        " use_gazebo:=true",
    ]
    robot_description_content = Command(xacro_cmd)
    robot_description = {"robot_description": robot_description_content}

    # robot_state_publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Start Gazebo (Harmonic / gz-sim).
    # The -r flag runs simulation on startup.
    gazebo = ExecuteProcess(
        cmd=[
            "gz", "sim", "-r", world,
        ],
        output="screen",
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
    )

    # Spawn the robot into Gazebo.
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-entity", "GHand",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.05",
        ],
        output="screen",
    )

    # Controller spawners.
    # gazebo_ros2_control creates the controller_manager inside Gazebo.
    jtc_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Build MoveIt configs (use the same config as moveit.launch.py).
    moveit_config = (
        MoveItConfigsBuilder("G5-A7-R", package_name="ghand_moveit_config")
        .robot_description_semantic(file_path="config/G5-A7-R.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )
    moveit_config.robot_description = {"robot_description": robot_description_content}

    # MoveGroup
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        additional_env={
            "DISPLAY": os.environ.get("DISPLAY", ""),
        },
    )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([FindPackageShare("ghand_moveit_config"), "config", "moveit.rviz"]),
        ],
        condition=IfCondition(use_rviz),
        output="both",
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
    )

    return LaunchDescription(declared_args + [
        gazebo,
        rsp,
        spawn_entity,
        jsb_spawner,
        jtc_spawner,
        move_group,
        rviz,
    ])
