# ghand_hardware_interface

`ros2_control` hardware interface for the GHand dexterous hand.

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
ros2 launch ghand_hardware_interface ghand.launch.py use_mock_hardware:=true

# EtherCAT (replace enx... with your EtherCAT interface name, see `ip link`)
ros2 launch ghand_hardware_interface ghand.launch.py profile:=ethercat_default ifname:=enx000000000000

# CANFD via the GHand-Cpp-SDK compatible ZQWL CDC adapter
ros2 launch ghand_hardware_interface ghand.launch.py profile:=canfd_left
```

### Dual hand

The dual description is one robot with `left_` and `right_` joint prefixes,
one controller manager, and one trajectory controller per hand. Single-hand
launches keep their original unprefixed names for compatibility.

```bash
# Safe mock test
ros2 launch ghand_hardware_interface dual_hand.launch.py \
  use_mock_hardware:=true

# Two independent RS485 devices
ros2 launch ghand_hardware_interface dual_hand.launch.py \
  use_mock_hardware:=false \
  left_profile:=rs485_left right_profile:=rs485_dual_right \
  left_device:=/dev/ttyUSB0 right_device:=/dev/ttyUSB1

# Two independent CANFD CDC adapters
ros2 launch ghand_hardware_interface dual_hand.launch.py \
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
