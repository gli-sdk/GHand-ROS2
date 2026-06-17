#!/usr/bin/env python3
"""Bridge rqt_joint_trajectory_controller topic to JTC action for real hardware.

rqt generates sliders for all 18 URDF joints, but the EtherCAT slave only
supports 13 active joints (DIP joints are passive). Directly sending 18-joint
trajectories to the joint_trajectory_controller causes the controller to fail
because the passive joints can never reach their targets.

This node subscribes to the trajectory topic published by rqt, filters the
trajectory down to the 13 active joints, and forwards the target point as a
FollowJointTrajectory action goal.  An explicit start point is intentionally
omitted: the current state is used as the start, avoiding URDF joint-limit
violations when a hardware zero (e.g. thumb_tmc_aa near 0 rad) does not match
the URDF lower limit (0.349 rad).
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# EtherCAT slave only supports these 13 active joints (URDF order, no DIP).
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


class RqtJtcBridge(Node):
    # Rate-limit goal forwarding: rqt publishes at 10 Hz, which preempts JTC
    # goals before real hardware can catch up, leading to state tolerance
    # violations. We forward at most 2 Hz and only when the target changed.
    FORWARD_PERIOD_SEC = 0.5
    MIN_POSITION_CHANGE = 0.02  # rad

    def __init__(self):
        super().__init__("rqt_jtc_bridge")

        self._current_positions = {}  # type: dict[str, float]
        self._pending_target = None  # type: list[float] | None
        self._last_sent_target = None  # type: list[float] | None

        self._joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10
        )
        self._trajectory_sub = self.create_subscription(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            self._trajectory_cb,
            10,
        )
        self._action_client = ActionClient(
            self, FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory"
        )
        self._forward_timer = self.create_timer(
            self.FORWARD_PERIOD_SEC, self._forward_pending_goal
        )

        self.get_logger().info("RQT -> JTC bridge started")

    def _joint_state_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in ACTIVE_JOINTS:
                self._current_positions[name] = pos

    def _filter_positions(self, point: JointTrajectoryPoint, joint_names: list) -> list:
        """Return positions for ACTIVE_JOINTS, defaulting to 0 if missing."""
        name_to_idx = {name: i for i, name in enumerate(joint_names)}
        positions = []
        for joint in ACTIVE_JOINTS:
            if joint in name_to_idx:
                positions.append(float(point.positions[name_to_idx[joint]]))
            else:
                positions.append(0.0)
        return positions

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if goal_handle is None:
            self.get_logger().error("Failed to send action goal (no handle)")
            return
        if not goal_handle.accepted:
            self.get_logger().warn("Action goal rejected by JTC")
            return
        self.get_logger().info("Action goal accepted by JTC")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        result = future.result().result
        if result.error_code != 0:
            self.get_logger().warn(
                f"Action goal finished with error_code={result.error_code}, "
                f"error_string='{result.error_string}'"
            )
        else:
            self.get_logger().info("Action goal finished successfully")

    def _trajectory_cb(self, msg: JointTrajectory):
        if not msg.points:
            return
        target = self._filter_positions(msg.points[-1], msg.joint_names)
        if self._pending_target is None or self._should_update(target):
            self._pending_target = target

    def _should_update(self, new_target: list) -> bool:
        """Return True if any active joint moved more than MIN_POSITION_CHANGE."""
        if self._pending_target is None:
            return True
        return any(
            abs(a - b) > self.MIN_POSITION_CHANGE
            for a, b in zip(new_target, self._pending_target)
        )

    def _forward_pending_goal(self):
        if self._pending_target is None:
            return
        if self._last_sent_target is not None:
            if all(
                abs(a - b) < self.MIN_POSITION_CHANGE
                for a, b in zip(self._pending_target, self._last_sent_target)
            ):
                return

        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("JTC action server not available")
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        goal.trajectory.joint_names = ACTIVE_JOINTS

        # Send only the target point; let JTC use the current state as the start.
        # Adding an explicit start point at the current position can violate
        # URDF joint limits (e.g. thumb_tmc_aa reads near 0 but its lower limit
        # is 0.349 rad), causing JTC to silently abort the whole trajectory.
        target_point = JointTrajectoryPoint()
        target_point.positions = [float(p) for p in self._pending_target]
        target_point.time_from_start = Duration(sec=2)

        goal.trajectory.points = [target_point]

        # Loose tolerances: real hardware is open-loop-ish at the trajectory level
        # and DIP joints are not driven, so strict path tracking is not required.
        goal.path_tolerance = [
            JointTolerance(name=joint, position=2.0, velocity=1000.0, acceleration=1000.0)
            for joint in ACTIVE_JOINTS
        ]
        goal.goal_tolerance = [
            JointTolerance(name=joint, position=2.0, velocity=1000.0, acceleration=1000.0)
            for joint in ACTIVE_JOINTS
        ]
        goal.goal_time_tolerance = Duration(sec=1)

        self._last_sent_target = self._pending_target[:]
        send_future = self._action_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)
        self.get_logger().debug("Forwarded rqt trajectory as 13-joint action goal")


def main(args=None):
    rclpy.init(args=args)
    node = RqtJtcBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
