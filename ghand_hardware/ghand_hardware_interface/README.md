# ghand_hardware_interface

`ros2_control` hardware interface for the GHand dexterous hand.

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
| CANFD (ZLG) | `libusbcanfd` + `libzuds` | `-DGHAND_ENABLE_CANFD=ON` |

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
```
