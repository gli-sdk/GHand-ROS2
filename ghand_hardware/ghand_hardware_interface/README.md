# ghand_hardware_interface

`ros2_control` hardware interface for the GHand dexterous hand.

This package owns the shared C++ plugin. Product joint/sensor resource Xacros
are owned by `ghand_bringup/config/<product_name>/`; shared hardware-selection
and connection macros are in `ghand_bringup/config/common/`.

`product_name:=ghandlite1` selects Lite1's 11-joint feedback layout and six
controlled joints. Modbus command blocks use four registers, and EtherCAT uses
a two-byte hand header followed by six eight-byte commands. Lite1 has no
independent position sensor; closed-loop, MIT and kp/kd interfaces are not
available. The SDK encoder reserves Lite1's loop bits even though its current
API guard calls the mode closed loop; this ROS adaptation emits the reserved
zero bits and does not expose a loop selector, following the confirmed device
capability. The SDK checkout is not modified or linked.

## Joint controller GUI

Start the hardware or MoveIt launch file first, then open the compatible joint
trajectory controller GUI with:

```bash
ros2 run ghand_hardware_interface ghand_joint_controller_gui
```

This project launcher works around the removed
`reload_controller_libraries` service in recent ROS 2 Jazzy
`controller_manager` releases. The upstream RQT executable may otherwise show
empty controller-manager and controller selectors.
When exactly one controller manager and one compatible trajectory controller
are running, the launcher selects both automatically and displays their joint
sliders.

## Overview

This package provides a `hardware_interface::SystemInterface` implementation
exported as `ghand_hardware_interface/GHandHardwareInterface`. It abstracts low-level
communication over EtherCAT, RS485, or CANFD and exposes standard
`ros2_control` command/state interfaces.

## Communication backends

The following backends are compiled in when their dependencies are detected at
configure time:

| Backend | Library | Build flag |
|---------|---------|------------|
| EtherCAT | SOEM | `-DGHAND_ENABLE_ETHERCAT=ON` |
| RS485 | libmodbus | auto-detected |
| CANFD (GHand-Cpp-SDK/ZQWL) | Linux CDC serial | always available |
| Legacy ZLG CANFD | `libusbcanfd` + `libzuds` | `-DGHAND_ENABLE_ZLG_CANFD=ON` |

By default `./build.sh` produces a mock-only library that does not require any
vendor hardware libraries. See the root `README.md` for installation
instructions for the optional backends.

## Plugin

The hardware plugin is declared in `hand_hardware_interface.xml` and loaded by
the controller manager through the `<ros2_control>` tag in the URDF.

## Dependencies

- `hardware_interface`
- `pluginlib`
- `rclcpp`
- `rclcpp_lifecycle`
- `libmodbus` (optional)
- `soem` (optional)

## Launch

```bash
# Mock hardware (no physical hand required)
ros2 launch ghand_bringup ghand.launch.py use_mock_hardware:=true

# EtherCAT (replace enx... with your EtherCAT interface name, see `ip link`)
ros2 launch ghand_bringup ghand.launch.py profile:=ethercat_default ifname:=enx000000000000

# CANFD via the GHand-Cpp-SDK compatible ZQWL CDC adapter
ros2 launch ghand_bringup ghand.launch.py profile:=canfd_left
```

### Dual hand

The dual description is one robot with `left_` and `right_` joint prefixes,
one controller manager, and one trajectory controller per hand. Single-hand
launches keep their original unprefixed names for compatibility.

```bash
# Safe mock test
ros2 launch ghand_bringup dual_hand.launch.py \
  use_mock_hardware:=true

# Two independent RS485 devices
ros2 launch ghand_bringup dual_hand.launch.py \
  use_mock_hardware:=false \
  left_profile:=rs485_left right_profile:=rs485_dual_right \
  left_device:=/dev/ttyUSB0 right_device:=/dev/ttyUSB1

# Two independent CANFD CDC adapters
ros2 launch ghand_bringup dual_hand.launch.py \
  use_mock_hardware:=false \
  left_profile:=canfd_left right_profile:=canfd_right \
  left_device:=/dev/ttyACM0:0 right_device:=/dev/ttyACM1:0
```

The dual trajectory actions are:

- `/left_joint_trajectory_controller/follow_joint_trajectory`
- `/right_joint_trajectory_controller/follow_joint_trajectory`

With both controllers running, the GUI controller selector contains one left
and one right entry:

```bash
ros2 run ghand_hardware_interface ghand_joint_controller_gui
```

### SDK control fields (real hardware)

Set `hardware.control_mode` in the selected profile to `position` (default),
`speed`/`velocity`, `current`, `mit`, or `custom`. The mode is fixed until the
backend is reconfigured. EtherCAT, RS485, ZQWL CANFD and legacy ZLG writes use
the same selection; invalid values fail initialization.

```yaml
hardware:
  # Keep the connection fields from your existing profile.
  control_mode: custom
```

Real-hardware joint command interfaces additionally expose `current_percent`,
`control_mode` (0 position, 1 speed, 2 current, 3 MIT), `loop_type` (0 semi-closed,
1 closed), `control_state` (0 disabled, 1 running, 2 stopped/holding), `kp` and
`kd`. Select `custom` to choose different modes or loop types per joint.
Outside CUSTOM, the joint mode must match the configured hand mode and the
loop type must be closed. Gains are signed 16-bit integer values.

Position commands use radians. In position mode, trajectory velocity remains
in rad/s and the hardware uses a 100% velocity limit; current defaults to a
100% limit. `current_percent` can explicitly set that limit to 0–100%.
In speed/current/MIT modes, velocity is a signed percentage and current is
clamped to -100–100%. Explicit `current_percent` takes precedence over the
legacy `effort` command. Position trajectory controllers are intended for
the default position mode; other modes need controllers claiming the relevant
percentage and SDK command interfaces. The new command interfaces start as
NaN until a controller writes them; do not initialize unused fields to zero.

Additional joint feedback interfaces are `current_percent`, `control_mode`
and `loop_type`. The existing `effort` feedback remains zero because current
percentage is not a measured torque. The `${prefix}hand` sensor exposes
`hand_state`, `temperature_raw`, `warning_0` through `warning_5`, `fault_0`
through `fault_11` and `critical_0` through `critical_5`. Temperature retains
the protocol integer until its physical unit is confirmed. Unavailable hand
feedback is NaN. RS485 reads the hand header with the joint block; legacy ZLG
does not provide this header or mode/loop feedback, so those values are NaN.
Mock and Gazebo interfaces remain unchanged.

Modbus writes stop and hold omitted active joints; explicitly requested
DISABLED joints remain disabled. EtherCAT retains earlier commands for
omitted joints, matching the SDK backend. Both codecs truncate angle targets
toward zero at 0.1-degree resolution.

These changes have offline codec coverage. Build the ROS2 package on Linux
before testing the device; the current Windows session does not verify the
ROS2 plugin or the slave PDO mapping.
