import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch argument: use_sim
    use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Start robot in Gazebo simulation.",
    )
    use_sim = LaunchConfiguration("use_sim")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("GHand_description"), "urdf", "G5-A7-R.urdf"]
            ),
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
    hand_msgs_lib = os.path.join(
        os.path.expanduser("~"), "GHand-Ros2", "install", "hand_msgs", "lib"
    )
    controller_manager_env["LD_LIBRARY_PATH"] = (
        hand_msgs_lib + ":" + controller_manager_env.get("LD_LIBRARY_PATH", "")
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

    nodes = [
        use_sim_arg,
        robot_state_pub_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        joint_status_broadcaster_spawner,
        tactile_broadcaster_spawner,
        joint_traj_controller_spawner,
    ]

    return LaunchDescription(nodes)
