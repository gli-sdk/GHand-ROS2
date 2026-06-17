#!/usr/bin/env python3
"""Example action client for moving the GHand through the JTC.

The EtherCAT slave only drives 13 active joints; DIP joints are passive.
This script sends FollowJointTrajectory action goals containing only those
13 joints.
"""

import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState

# EtherCAT slave only supports these 13 active joints (no DIP).
ACTIVE_JOINTS = [
    "thumb_tmc_ps",
    "thumb_tmc_aa",
    "thumb_tmc_fe",
    "thumb_mcp",
    "index_mcp_aa",
    "index_mcp",
    "index_pip",
    "middle_mcp",
    "middle_pip",
    "ring_mcp",
    "ring_pip",
    "little_mcp",
    "little_pip",
]

# Open hand: all active joints at zero.
OPEN_POSITIONS = [0.0] * len(ACTIVE_JOINTS)

# Fist: bend all active joints.
FIST_POSITIONS = [0.5] * len(ACTIVE_JOINTS)


def get_current_positions(node):
    """Read current active-joint positions from /joint_states."""
    current = {}
    received = []

    def callback(msg):
        if not msg.position:
            return
        for name, pos in zip(msg.name, msg.position):
            if name in ACTIVE_JOINTS:
                current[name] = pos
        if len(current) == len(ACTIVE_JOINTS):
            received.append(True)

    sub = node.create_subscription(JointState, "/joint_states", callback, 10)
    timeout = node.get_clock().now() + rclpy.duration.Duration(seconds=3.0)
    while len(current) < len(ACTIVE_JOINTS) and node.get_clock().now() < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    sub.destroy()

    if len(current) < len(ACTIVE_JOINTS):
        node.get_logger().warn("Could not read all joint states, using zeros")
        return [0.0] * len(ACTIVE_JOINTS)

    return [current[j] for j in ACTIVE_JOINTS]


def send_goal(node, client, target_positions, duration_sec=2.0):
    """Send a trajectory goal and wait for the result."""
    current_positions = get_current_positions(node)

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = JointTrajectory()
    goal.trajectory.header.stamp = node.get_clock().now().to_msg()
    goal.trajectory.joint_names = ACTIVE_JOINTS

    start_point = JointTrajectoryPoint()
    start_point.positions = current_positions
    start_point.time_from_start = Duration(sec=0)

    target_point = JointTrajectoryPoint()
    target_point.positions = target_positions
    target_point.time_from_start = Duration(sec=int(duration_sec))

    goal.trajectory.points = [start_point, target_point]

    # Loose tolerances avoid path-tolerance violations on real hardware.
    goal.path_tolerance = [
        JointTolerance(name=j, position=2.0, velocity=1000.0, acceleration=1000.0)
        for j in ACTIVE_JOINTS
    ]
    goal.goal_tolerance = [
        JointTolerance(name=j, position=2.0, velocity=1000.0, acceleration=1000.0)
        for j in ACTIVE_JOINTS
    ]
    goal.goal_time_tolerance = Duration(sec=1)

    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)

    goal_handle = future.result()
    if not goal_handle or not goal_handle.accepted:
        node.get_logger().error("Goal rejected")
        return False

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result().result
    if result.error_code != 0:
        node.get_logger().warn(f"Goal failed: {result.error_code}, {result.error_string}")
        return False
    return True


def main():
    rclpy.init()
    node = rclpy.create_node("hand_fist_open_loop")
    client = ActionClient(
        node, FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory"
    )

    if not client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error("Action server not available")
        return

    node.get_logger().info("Starting fist-open loop. Press Ctrl-C to stop.")
    loop_count = 0

    try:
        while rclpy.ok():
            loop_count += 1
            node.get_logger().info(f"=== Loop {loop_count}: fist ===")
            if not send_goal(node, client, FIST_POSITIONS):
                break

            node.get_logger().info(f"=== Loop {loop_count}: open ===")
            if not send_goal(node, client, OPEN_POSITIONS):
                break
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user")

    node.get_logger().info("Exiting")


if __name__ == "__main__":
    main()
