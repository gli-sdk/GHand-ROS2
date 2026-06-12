"""GHand MoveIt 2 launch file.

参数:
  use_sim:   true=mock hardware, false=真实硬件 (默认: true)
  use_rviz:  是否启动 RViz (默认: true)
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
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim")
    use_rviz = LaunchConfiguration("use_rviz")

    declared_args = [
        DeclareLaunchArgument("use_sim", default_value="true",
                              description="Use mock hardware when true"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Launch RViz when true"),
    ]

    # URDF via xacro with mock/real switching
    xacro_cmd = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("GHand_description"), "urdf", "G5-A7-R.urdf"]),
        " use_mock_hardware:=",
        PythonExpression(["'true' if '", use_sim, "' == 'true' else 'false'"]),
    ]
    robot_description_content = Command(xacro_cmd)

    # Build MoveIt configs (skip robot_description — inject manually after builder)
    moveit_config = (
        MoveItConfigsBuilder("G5-A7-R", package_name="GHand_moveit_config")
        .robot_description_semantic(file_path="config/G5-A7-R.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )
    moveit_config.robot_description = {"robot_description": robot_description_content}

    # robot_state_publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description_content}],
    )

    # controller_manager — env fix for hand_msgs
    cm_env = os.environ.copy()
    hand_msgs_lib = os.path.join(get_package_prefix("hand_msgs"), "lib")
    cm_env["LD_LIBRARY_PATH"] = hand_msgs_lib + ":" + cm_env.get("LD_LIBRARY_PATH", "")

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_content},
            PathJoinSubstitution([FindPackageShare("hand_bringup"), "config", "ros2_control.yaml"]),
            PathJoinSubstitution([FindPackageShare("hand_bringup"), "config", "controllers.yaml"]),
        ],
        output="both",
        env=cm_env,
    )

    # Controller spawners
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
    jstatus_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_status_broadcaster", "--controller-manager", "/controller_manager"],
    )
    tactile_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["tactile_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Build env with all library paths (vendor + ROS)
    mg_env = os.environ.copy()
    vendor_lib_paths = (
        "/opt/ros/jazzy/lib:"
        "/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:"
        "/opt/ros/jazzy/opt/gz_math_vendor/lib:"
        "/opt/ros/jazzy/opt/gz_utils_vendor/lib:"
        "/opt/ros/jazzy/opt/gz_common_vendor/lib:"
        "/opt/ros/jazzy/opt/sdformat_vendor/lib"
    )
    mg_env["LD_LIBRARY_PATH"] = vendor_lib_paths + ":" + mg_env.get("LD_LIBRARY_PATH", "")

    # MoveGroup
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        additional_env={
            "DISPLAY": os.environ.get("DISPLAY", ""),
            "LD_LIBRARY_PATH": mg_env["LD_LIBRARY_PATH"],
        },
    )

    # RViz — use ExecuteProcess to bypass Node env filtering
    rviz = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            "VENDOR_LIBS=$(find /opt/ros/jazzy/opt -maxdepth 2 -name lib -type d | tr '\\n' ':'); "
            "export LD_LIBRARY_PATH=\"$VENDOR_LIBS$LD_LIBRARY_PATH\"; "
            "rviz2 -d $(ros2 pkg prefix GHand_moveit_config)/share/GHand_moveit_config/config/moveit.rviz",
        ],
        output="both",
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(declared_args + [
        rsp,
        ros2_control_node,
        jsb_spawner,
        jstatus_spawner,
        tactile_spawner,
        jtc_spawner,
        move_group,
        rviz,
    ])
