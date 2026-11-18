# GHand-ROS2 Interface Reference

[简体中文](ROS2_INTERFACE_REFERENCE_CN.md)

This document freezes the public ROS 2 API exposed by the core
`GHand-ROS2` repository. Demos and downstream applications should depend on
these names instead of controller-private defaults or README-only examples.

## Packages

- `ghand_description`: URDF generation and mesh installation.
- `ghand_hardware_interface`: shared `ros2_control` hardware plugin.
- `ghand_bringup`: scene orchestration and hardware launch files, controller configuration and communication profiles.
- `ghand_controllers`: GHand-specific status and tactile broadcasters.
- `ghand_msgs`: custom GHand status and tactile message definitions.
- `ghand_moveit_config`: MoveIt 2 configuration.
- `ghand_gazebo`: Gazebo Fortress simulation integration.

`ghand_controllers` and `ghand_msgs` are the public package names for GHand
custom broadcasters and messages.

## Launch Files

Recommended public entry: `ros2 launch ghand_bringup start.launch.py`.

- `backend:=preview`: planning/display only; execution disabled, no controller manager.
- `backend:=gazebo` (default): physics and controllers, optional MoveIt.
- `backend:=hardware`: physical controllers, optional MoveIt.
- `hand:=left/right/dual`, `product_name:=ghand5/ghandlite1`.
- `planning:=true/false` (default true); preview requires true.
- `use_rviz:=true/false` (default true).
- Hardware connection options and migration: [bringup guide](../ghand_bringup/README.md).

Physical hardware (these entries connect to devices):

- Single hand: `ros2 launch ghand_bringup ghand.launch.py`
- Dual hand: `ros2 launch ghand_bringup dual_hand.launch.py`

MoveIt planning preview (no execution; `mode:=attached` connects to existing physical bringup):

- Single hand: `ros2 launch ghand_moveit_config moveit.launch.py`
- Dual hand: `ros2 launch ghand_moveit_config dual_moveit.launch.py`

Gazebo:

- Single hand: `ros2 launch ghand_gazebo ghand_gazebo.launch.py`
- Dual hand: `ros2 launch ghand_gazebo dual_hand_gazebo.launch.py`

## Launch Arguments

All entry points accept `product_name`, default `ghand5`. Available choices
come from installed `ghand_bringup/config/<product_name>/controllers.yaml` files.
Configured products are `ghand5` and `ghandlite1`. Both hands in a dual launch
use the selected product. Communication profiles remain independent of product
selection.

Lite1 has 11 feedback joints and six active joints (`THUMB_TMC_FE`,
`THUMB_TMC_AA`, `FF_MCP`, `MF_MCP`, `RF_MCP`, `LF_MCP`). `THUMB_MCP` and
the four `*_IP` joints are passive. Lite1 uses semi-closed control; closed-loop,
MIT and kp/kd command interfaces are unavailable. Its hand fault block contains
six bytes; unused exported fault slots are zero. Its tactile layout is
38/31/31/31/31 according to the SDK baseline `6651e46`.

Product configurations live under `config/<product_name>/` inside bringup,
Gazebo and MoveIt. Shared hardware macros live in `ghand_bringup/config/common/`.
Installed descriptions and meshes live under `ghand_description/urdf/<product_name>/`
and `ghand_description/meshes/<product_name>/`. Direct users of old configuration
paths must migrate; public launch filenames and default controller names remain
unchanged. Rebuild into a fresh install directory to avoid stale old-path files.

Single-hand hardware arguments:

- `hand_side`: `left` or `right`; default `right`.
- `use_rviz`: `true` or `false`; default `true`.
- `profile`: profile file stem under `ghand_bringup/config/profiles`; default `ethercat_default`.
- `communication_type`: optional profile override: `ethercat`, `rs485`, or `canfd`.
- `device`: optional serial/CAN device override.
- `baud_rate`: optional RS485/CANFD arbitration bitrate override.
- `dbit_rate`: optional CANFD data-phase bitrate override.
- `slave_id`: optional Modbus/CANFD node ID override.
- `ifname`: optional EtherCAT interface override.

Dual-hand hardware arguments:

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
- `velocity_percent_controller` inactive by default.
- `current_percent_controller` inactive by default.

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
- Real hardware velocity: `velocity_percent` in percent; no estimated standard velocity is published.
- Real hardware current: `current_percent` in percent; no physical effort/torque is inferred.
- HardwareInterface owns ROS-to-SDK unit conversion.

## Description Reproducibility

`ghand_description` sources URDF and mesh assets from the `ghand_assets`
repository. Clone its default branch from either Gitee or GitHub into the
workspace as `ghand_assets`.

```bash
cd <workspace>/src
git clone -b humble https://gitee.com/glitech/ghand-ros2.git
git clone --depth 1 \
  https://gitee.com/glitech/ghand-assets.git ghand_assets
# GitHub alternative:
# git clone --depth 1 \
#   https://github.com/gli-sdk/GHand-Assets.git ghand_assets
cd <workspace>
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## Smoke Test

After launching hardware or Gazebo:

```bash
ros2 control list_controllers
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
ros2 topic list
ros2 topic echo /joint_states
```
