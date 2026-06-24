"""GHand Gazebo simulation launch file.

Parameters:
  use_rviz: 是否启动 RViz (默认: true)
  world:    Gazebo world 文件路径 (默认: empty.world)
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, SetParameter
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

    # --- Environment setup for Gazebo Sim (gz) ---
    # 1. Resource path: gz resolves mesh URIs (package://ghand_description/... is
    #    converted to model://ghand_description/...) against GZ_SIM_RESOURCE_PATH.
    #    Point it at the parent of ghand_description's share dir so model://ghand_description
    #    resolves to the installed meshes.
    # 2. System plugin path: gz_ros2_control-system lives in the ROS lib dir.
    ghand_desc_share_parent = os.path.dirname(get_package_share_directory("ghand_description"))
    ros_lib = os.path.join(get_package_prefix("gz_ros2_control"), "lib")
    # Architecture-specific ROS lib dir (contains libruckig.so on amd64).
    ros_lib_x86_64 = os.path.join(
        get_package_prefix("moveit_ros_move_group"), "lib", "x86_64-linux-gnu"
    )
    rviz_ogre_lib = os.path.join(
        get_package_prefix("rviz_ogre_vendor"), "opt", "rviz_ogre_vendor", "lib"
    )

    env_actions = [
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", ghand_desc_share_parent),
        AppendEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", ros_lib),
        AppendEnvironmentVariable("LD_LIBRARY_PATH", ros_lib_x86_64),
        AppendEnvironmentVariable("LD_LIBRARY_PATH", rviz_ogre_lib),
    ]

    # NVIDIA PRIME workaround: gz's OGRE2 render engine picks the mesa EGL vendor by
    # default and segfaults on hybrid NVIDIA setups. Force the NVIDIA EGL/GLX vendor.
    # Only applied when the NVIDIA glvnd config is present, so this is a no-op on
    # non-NVIDIA machines.
    nvidia_egl = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
    if os.path.exists(nvidia_egl):
        env_actions += [
            SetEnvironmentVariable("__NV_PRIME_RENDER_OFFLOAD", "1"),
            SetEnvironmentVariable("__GLX_VENDOR_LIBRARY_NAME", "nvidia"),
            SetEnvironmentVariable("__EGL_VENDOR_LIBRARY_FILENAMES", nvidia_egl),
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

    # Clock bridge: publish gz's simulation time to /clock so ROS nodes can use
    # use_sim_time. Bridge `clock` (gz) -> `/clock` (ROS).  Wait until the gz
    # process has started so the Gazebo Transport publisher is discoverable.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )
    clock_bridge_starter = RegisterEventHandler(
        OnProcessStart(target_action=gazebo, on_start=[clock_bridge])
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
        parameters=[moveit_config.to_dict()],
        condition=IfCondition(use_rviz),
        output="both",
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
    )

    return LaunchDescription(declared_args + env_actions + [
        SetParameter(name="use_sim_time", value=True),
        gazebo,
        clock_bridge_starter,
        rsp,
        spawn_entity,
        jsb_spawner,
        jtc_spawner,
        move_group,
        rviz,
    ])
