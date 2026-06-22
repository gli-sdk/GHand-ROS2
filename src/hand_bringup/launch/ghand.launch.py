import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch argument: use_mock_hardware
    use_mock_hardware_arg = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="false",
        description="Use mock hardware instead of real GHand hardware.",
    )
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    # Launch arguments: communication backend selection
    communication_type_arg = DeclareLaunchArgument(
        "communication_type",
        default_value="ethercat",
        description="Communication backend: ethercat, rs485, or canfd.",
    )
    communication_type = LaunchConfiguration("communication_type")

    device_arg = DeclareLaunchArgument(
        "device",
        default_value="/dev/ttyUSB0",
        description="Serial/CAN device for rs485 or canfd backend.",
    )
    device = LaunchConfiguration("device")

    baud_rate_arg = DeclareLaunchArgument(
        "baud_rate",
        default_value="1000000",
        description="Baud rate for rs485/canfd backend.",
    )
    baud_rate = LaunchConfiguration("baud_rate")

    dbit_rate_arg = DeclareLaunchArgument(
        "dbit_rate",
        default_value="5000000",
        description="CANFD data phase bit rate (only used when communication_type=canfd).",
    )
    dbit_rate = LaunchConfiguration("dbit_rate")

    slave_id_arg = DeclareLaunchArgument(
        "slave_id",
        default_value="49",
        description="Modbus slave ID or CAN device ID.",
    )
    slave_id = LaunchConfiguration("slave_id")

    # Get URDF via xacro, switching between mock and real hardware
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ghand_description"), "urdf", "G5-A7-R.urdf"]
            ),
            " use_mock_hardware:=",
            use_mock_hardware,
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
            "export LD_LIBRARY_PATH=\"/opt/ros/jazzy/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH\"; for d in /opt/ros/jazzy/opt/*/lib; do export LD_LIBRARY_PATH=\"$d:$LD_LIBRARY_PATH\"; done && rviz2 -d $(ros2 pkg prefix hand_bringup)/share/hand_bringup/config/ghand.rviz",
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
        communication_type_arg,
        device_arg,
        baud_rate_arg,
        dbit_rate_arg,
        slave_id_arg,
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
