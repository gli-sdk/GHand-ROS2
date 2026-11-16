# GHand-ROS2 Interface Reference

This document freezes the public ROS 2 API exposed by the core
`GHand-ROS2` repository. Demos and downstream applications should depend on
these names instead of controller-private defaults or README-only examples.

## Packages

- `ghand_description`: URDF generation and mesh installation.
- `ghand_hardware_interface`: `ros2_control` hardware plugin and interface Xacro macro.
- `ghand_bringup`: real/mock launch files, controller configuration and communication profiles.
- `ghand_controllers`: GHand-specific status and tactile broadcasters.
- `ghand_msgs`: custom GHand status and tactile message definitions.
- `ghand_moveit_config`: MoveIt 2 configuration.
- `ghand_gazebo`: Gazebo Harmonic simulation integration.

`ghand_controllers` and `ghand_msgs` are the public package names for GHand
custom broadcasters and messages.

## Launch Files

Hardware and mock:

- Single hand: `ros2 launch ghand_bringup ghand.launch.py`
- Dual hand: `ros2 launch ghand_bringup dual_hand.launch.py`

MoveIt:

- Single hand: `ros2 launch ghand_moveit_config moveit.launch.py`
- Dual hand: `ros2 launch ghand_moveit_config dual_moveit.launch.py`

Gazebo:

- Single hand: `ros2 launch ghand_gazebo ghand_gazebo.launch.py`
- Dual hand: `ros2 launch ghand_gazebo dual_hand_gazebo.launch.py`

## Launch Arguments

Single-hand hardware arguments:

- `hand_side`: `left` or `right`; default `right`.
- `use_mock_hardware`: `true` or `false`; default `false`.
- `profile`: profile file stem under `ghand_bringup/config/profiles`; default `ethercat_default`.
- `communication_type`: optional profile override: `ethercat`, `rs485`, or `canfd`.
- `device`: optional serial/CAN device override.
- `baud_rate`: optional RS485/CANFD arbitration bitrate override.
- `dbit_rate`: optional CANFD data-phase bitrate override.
- `slave_id`: optional Modbus/CANFD node ID override.
- `ifname`: optional EtherCAT interface override.

Dual-hand hardware arguments:

- `use_mock_hardware`: `true` or `false`; default `true`.
- `use_rviz`: `true` or `false`; default `true`.
- `left_profile`: default `rs485_left`.
- `right_profile`: default `rs485_dual_right`.
- `left_device`, `right_device`: optional per-hand device overrides.
- `left_ifname`, `right_ifname`: optional per-hand EtherCAT interface overrides.

## Controllers And Actions

Single-hand controllers:

- `joint_state_broadcaster`
- `joint_status_broadcaster`
- `tactile_broadcaster`
- `joint_trajectory_controller`
- `position_controller` inactive by default.
- `velocity_controller` inactive by default.
- `effort_controller` inactive by default.

Dual-hand controllers:

- `joint_state_broadcaster`
- `left_joint_status_broadcaster`
- `right_joint_status_broadcaster`
- `left_tactile_broadcaster`
- `right_tactile_broadcaster`
- `left_joint_trajectory_controller`
- `right_joint_trajectory_controller`

Trajectory actions:

- Single hand: `/joint_trajectory_controller/follow_joint_trajectory`
- Dual left hand: `/left_joint_trajectory_controller/follow_joint_trajectory`
- Dual right hand: `/right_joint_trajectory_controller/follow_joint_trajectory`

Action type:

- `control_msgs/action/FollowJointTrajectory`

## Topics

Single-hand public topics:

- `/joint_states`: `sensor_msgs/msg/JointState`
- `/joint_status`: `ghand_msgs/msg/JointStatus`
- `/tactile_data`: `ghand_msgs/msg/TactileData`
- `/joint_trajectory_controller/joint_trajectory`: `trajectory_msgs/msg/JointTrajectory`

Dual-hand public topics:

- `/joint_states`: `sensor_msgs/msg/JointState`
- `/left/joint_status`: `ghand_msgs/msg/JointStatus`
- `/right/joint_status`: `ghand_msgs/msg/JointStatus`
- `/left/tactile_data`: `ghand_msgs/msg/TactileData`
- `/right/tactile_data`: `ghand_msgs/msg/TactileData`
- `/left_joint_trajectory_controller/joint_trajectory`: `trajectory_msgs/msg/JointTrajectory`
- `/right_joint_trajectory_controller/joint_trajectory`: `trajectory_msgs/msg/JointTrajectory`

The status and tactile broadcasters publish to their `topic` parameter. The
core configuration sets that parameter to the public names above.

## MoveIt Groups

- Single hand SRDF: `hand`.
- Dual hand SRDF: `left_hand`, `right_hand`, `dual_hands`.

## Joint Names

Single-hand actuated trajectory joints, in controller order:

```text
Thumb_MCP
Thumb_TMC_FE
Thumb_TMC_AA
Thumb_TMC_PS
FF_PIP
FF_MCP
FF_MCP_AA
MF_PIP
MF_MCP
RF_PIP
RF_MCP
LF_PIP
LF_MCP
```

Single-hand status joints include the actuated joints plus passive/mimic DIP
and thumb IP joints:

```text
Thumb_IP
Thumb_MCP
Thumb_TMC_FE
Thumb_TMC_AA
Thumb_TMC_PS
FF_DIP
FF_PIP
FF_MCP
FF_MCP_AA
MF_DIP
MF_PIP
MF_MCP
RF_DIP
RF_PIP
RF_MCP
LF_DIP
LF_PIP
LF_MCP
```

Dual-hand joint names use the same names prefixed with `left_` or `right_`.

## Tactile Names

Single-hand tactile sensor names are `thumb`, `index`, `middle`, `ring`, and
`little`. Dual-hand names use `left_` or `right_` prefixes. Point counts are
`38` for thumb and `31` for each finger.

## Units

- ROS joint position: radians.
- ROS joint velocity: radians per second.
- ROS effort: backend-specific effort command exposed through `ros2_control`.
- HardwareInterface owns ROS-to-SDK unit conversion.

## Description Reproducibility

`ghand_description` sources URDF and mesh assets from the `ghand_assets`
repository imported by `ghand.repos`. The current asset revision is pinned to
`V1.0.0`.

```bash
cd <workspace>/src
git clone https://gitee.com/glitech/ghand-ros2.git
vcs import . < ghand-ros2/ghand.repos --recursive --skip-existing
cd <workspace>
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## Smoke Test

After launching hardware, mock hardware, or Gazebo:

```bash
ros2 control list_controllers
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
ros2 topic list
ros2 topic echo /joint_states
```
