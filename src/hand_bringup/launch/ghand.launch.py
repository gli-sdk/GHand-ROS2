import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def resolve_profile(context):
    """Resolve profile file path and verify it exists."""
    profile = perform_substitutions(context, [LaunchConfiguration("profile")])
    profile_file = os.path.join(
        get_package_share_directory("hand_bringup"),
        "config",
        "profiles",
        f"{profile}.yaml",
    )
    if not os.path.exists(profile_file):
        available = []
        profiles_dir = os.path.dirname(profile_file)
        if os.path.isdir(profiles_dir):
            available = sorted([
                f.replace(".yaml", "")
                for f in os.listdir(profiles_dir)
                if f.endswith(".yaml")
            ])
        raise RuntimeError(
            f"Communication profile not found: {profile_file}\n"
            f"Requested profile: '{profile}'\n"
            f"Available profiles: {', '.join(available) if available else '<none>'}"
        )
    return []


def generate_launch_description():
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
        description="Communication profile under hand_bringup/config/profiles/.",
    )

    # Legacy launch arguments (empty defaults mean "use profile value").
    # When provided, they override the corresponding profile field.
    communication_type_arg = DeclareLaunchArgument(
        "communication_type",
        default_value="",
        description="Override: communication backend (ethercat, rs485, canfd, zqwl_canfd).",
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
        description="Override: Baud rate for rs485 or CANFD arbitration phase.",
    )
    baud_rate = LaunchConfiguration("baud_rate")

    dbit_rate_arg = DeclareLaunchArgument(
        "dbit_rate",
        default_value="",
        description="Override: CANFD data phase bit rate (legacy name for data_rate).",
    )
    dbit_rate = LaunchConfiguration("dbit_rate")

    slave_id_arg = DeclareLaunchArgument(
        "slave_id",
        default_value="",
        description="Override: Modbus slave ID / CAN device ID (legacy name for node_id).",
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
                [FindPackageShare("ghand_description"), "urdf", "G5-A7-R.urdf"]
            ),
            " use_mock_hardware:=",
            use_mock_hardware,
            " profile_file:=",
            PathJoinSubstitution(
                [FindPackageShare("hand_bringup"), "config", "profiles", [LaunchConfiguration("profile"), ".yaml"]]
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

    # Preserve full environment for controller_manager so it can resolve
    # hand_msgs shared libraries while keeping HOME, PATH, etc.
    controller_manager_env = os.environ.copy()
    hand_msgs_lib = os.path.join(get_package_prefix("hand_msgs"), "lib")
    vendor_zlg_lib = os.path.join(
        get_package_prefix("ghand_hardware_interface"), "..", "..", "vendor",
        "usbcanfd_libusb_x64_1.0.12_251029"
    )
    vendor_zlg_lib = os.path.normpath(vendor_zlg_lib)
    controller_manager_env["LD_LIBRARY_PATH"] = (
        hand_msgs_lib + ":" + vendor_zlg_lib + ":" + controller_manager_env.get("LD_LIBRARY_PATH", "")
    )
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution(
                [FindPackageShare("hand_bringup"), "config", "ros2_control.yaml"]
            ),
            PathJoinSubstitution(
                [FindPackageShare("hand_bringup"), "config", "controllers.yaml"]
            ),
        ],
        output="both",
        env=controller_manager_env,
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    joint_status_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_status_broadcaster", "--controller-manager", "/controller_manager"],
    )

    tactile_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["tactile_broadcaster", "--controller-manager", "/controller_manager"],
    )

    joint_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager", "--inactive"],
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--controller-manager", "/controller_manager", "--inactive"],
    )

    effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_controller", "--controller-manager", "/controller_manager", "--inactive"],
    )

    rviz_process = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            'export LD_LIBRARY_PATH="/opt/ros/jazzy/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"; for d in /opt/ros/jazzy/opt/*/lib; do export LD_LIBRARY_PATH="$d:$LD_LIBRARY_PATH"; done && rviz2 -d $(ros2 pkg prefix hand_bringup)/share/hand_bringup/config/ghand.rviz',
        ],
        output="both",
    )

    # Ensure the vendored ZLG USB-CANFD library is discoverable at runtime.
    append_vendor_lib = AppendEnvironmentVariable(
        "LD_LIBRARY_PATH", vendor_zlg_lib
    )

    nodes = [
        append_vendor_lib,
        use_mock_hardware_arg,
        profile_arg,
        communication_type_arg,
        device_arg,
        baud_rate_arg,
        dbit_rate_arg,
        slave_id_arg,
        ifname_arg,
        OpaqueFunction(function=resolve_profile),
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
