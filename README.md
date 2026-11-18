# GHand-ROS2

[简体中文](README_CN.md) · [Installation](docs/INSTALLATION_EN.md) · [Interface reference](docs/ROS2_INTERFACE_REFERENCE.md)

ROS 2 Humble control stack for GHand5 and Lite1, with single-hand and same-product dual-hand configurations.
Choose **MoveIt planning preview, Gazebo physics simulation, or physical hardware control**.

![MoveIt planning](docs/moveit_demo.gif)
![Gazebo simulation](docs/gazebo_demo.gif)

## Environment and products

Ubuntu 22.04, ROS 2 Humble, C++17 and Python 3.10. Physics simulation uses Gazebo Fortress.
Follow the [installation guide](docs/INSTALLATION_EN.md) to clone the source and separate model assets, install dependencies and build.

| Product | `product_name` | Feedback / active joints | Control capabilities |
| --- | --- | --- | --- |
| GHand5 | `ghand5` (default) | 18 / 13 | See [hardware interface](ghand_hardware/ghand_hardware_interface/README.md) |
| Lite1 | `ghandlite1` | 11 / 6 | Semi-closed control; no closed-loop, MIT or kp/kd interfaces |

Both products provide single/dual, Gazebo and MoveIt configurations. Configuration availability does not imply hardware validation.
Lite1 tactile resources follow current SDK metadata; actual support requires device/firmware confirmation and hardware validation.

## Choose a scenario

| Scenario | Argument | Trajectory execution | Contact physics |
| --- | --- | --- | --- |
| MoveIt preview | `backend:=preview` | Planning and display only | No |
| Gazebo | `backend:=gazebo` | In Gazebo | Yes |
| Hardware | `backend:=hardware` | On the physical hand | Real contact |

Use `ghand_bringup start.launch.py` for all scenarios. Defaults: Gazebo, right hand, MoveIt enabled.
Run one scene at a time; exit with Ctrl+C before switching to avoid duplicate nodes and controllers.

Load the environment in each terminal:

```bash
source /opt/ros/humble/setup.bash
source <workspace>/install/setup.bash
```

### 1. MoveIt planning preview

```bash
ros2 launch ghand_bringup start.launch.py backend:=preview
```

Choose a goal in RViz MotionPlanning and click **Plan** to display the trajectory.
Preview publishes a fixed initial joint state, starts no controllers and disables execution.
Do not use Execute or Plan & Execute. Use Gazebo or hardware to validate execution.

### 2. Gazebo physics simulation

```bash
ros2 launch ghand_bringup start.launch.py backend:=gazebo
```

Starts Gazebo, controllers, MoveIt and RViz. Use **Plan & Execute** in RViz to execute in Gazebo.
Add `planning:=false` for Topic/Action control without MoveIt.

Verify from another sourced terminal:

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

`joint_state_broadcaster` and `joint_trajectory_controller` should be `active`.

### 3. Physical hardware

Complete the device, permission and wiring checks in the [hardware guide](docs/HARDWARE_EN.md).
For an RS485 right hand, replace the device path with the actual port:

```bash
ros2 launch ghand_bringup start.launch.py \
  backend:=hardware hand:=right planning:=false \
  profile:=rs485_right device:=/dev/ttyUSB0
```

Set `planning:=true` to enable MoveIt planning and execution. Physical hardware requires explicit `backend:=hardware`.

```bash
ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 topic echo /joint_status --once
```

Confirm that the hardware and trajectory controller are `active` and feedback is valid before sending motion commands.
See the [hardware guide](docs/HARDWARE_EN.md) for CANFD and EtherCAT.

## Products, left/right and dual hands

These selections apply to all three scenarios:

```bash
# Lite1 left-hand planning preview
ros2 launch ghand_bringup start.launch.py \
  backend:=preview product_name:=ghandlite1 hand:=left

# Same-product dual-hand Gazebo simulation
ros2 launch ghand_bringup start.launch.py backend:=gazebo hand:=dual

# Dual physical hands on separate RS485 ports
ros2 launch ghand_bringup start.launch.py \
  backend:=hardware hand:=dual planning:=false \
  left_profile:=rs485_left right_profile:=rs485_dual_right \
  left_device:=/dev/ttyUSB0 right_device:=/dev/ttyUSB1
```

## Control and feedback

Gazebo and hardware share the default trajectory API. Joint positions and trajectories use radians.

| Interface | Single-hand path |
| --- | --- |
| Joint feedback | `/joint_states` |
| Trajectory Action (recommended) | `/joint_trajectory_controller/follow_joint_trajectory` |
| Trajectory Topic | `/joint_trajectory_controller/joint_trajectory` |
| Hardware status / tactile feedback | `/joint_status` / `/tactile_data` |

Dual joints use `left_` and `right_` prefixes. Actions are
`/left_joint_trajectory_controller/follow_joint_trajectory` and
`/right_joint_trajectory_controller/follow_joint_trajectory`.
Hardware-specific feedback is under `/left/` and `/right/`.

See the [interface reference](docs/ROS2_INTERFACE_REFERENCE.md) for direct position, velocity/current percentage control, controller switching, message fields and product restrictions.

## Documentation and maintenance

- [Installation](docs/INSTALLATION_EN.md) · [Hardware connection](docs/HARDWARE_EN.md)
- [Launch arguments and migration](ghand_bringup/README.md)
- [MoveIt preview](ghand_moveit_config/README.md) · [Gazebo](ghand_gazebo/README.md)
- [Interface reference](docs/ROS2_INTERFACE_REFERENCE.md) · [Changelog](CHANGELOG.md)
- [Official documentation](https://fcnzogxju7xr.feishu.cn/docx/AhZ6ds2iCoguaAxIzBxciYHinNo) · [GLI SDK](https://github.com/gli-sdk)

Launch output uses standard ROS logging. Complete logs are saved in the directory printed by launch.
Rebuild and source the workspace after updates. Runtime development uses Gazebo or hardware; the mock backend has been removed.

Report issues in this repository or contact support@glitech.com. License: [Apache 2.0](LICENSE).

Physical hardware links the bundled GHand C++ SDK dynamic library: run `build.sh`. No local SDK source checkout is required. Use `build.sh --simulation` for preview/Gazebo without the SDK.
