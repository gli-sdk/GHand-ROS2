import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution


def generate_launch_description():
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

    # Delay controllers until controller_manager is up
    delay_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_manager_node,
            on_exit=[
                joint_state_broadcaster_spawner,
                joint_status_broadcaster_spawner,
                tactile_broadcaster_spawner,
                joint_traj_controller_spawner,
            ],
        )
    )

    nodes = [
        robot_state_pub_node,
        controller_manager_node,
        delay_controllers,
    ]

    return LaunchDescription(nodes)
