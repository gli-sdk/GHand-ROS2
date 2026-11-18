# ghand_bringup

Public scene orchestration and physical hardware control.

## Public entry point

```bash
ros2 launch ghand_bringup start.launch.py backend:=preview
ros2 launch ghand_bringup start.launch.py backend:=mock use_rviz:=false
ros2 launch ghand_bringup start.launch.py backend:=gazebo
ros2 launch ghand_bringup start.launch.py backend:=hardware planning:=false profile:=rs485_right device:=/dev/ttyUSB0
```

| Argument | Default | Meaning |
| --- | --- | --- |
| `backend` | `gazebo` | `preview`, `mock`, `gazebo`, `hardware` |
| `product_name` | `ghand5` | `ghand5`, `ghandlite1` |
| `hand` | `right` | `left`, `right`, `dual` |
| `planning` | `true` | Enable MoveIt; preview requires true |
| `use_rviz` | `true` | Show RViz |
| `profile` | empty | Hardware connection; resolves to `rs485_<hand>` for a single hand |
| `device`, `ifname` | empty | Override serial/CAN device or EtherCAT interface |
| `left_profile`, `right_profile` | `rs485_left`, `rs485_dual_right` | Dual hardware connections |
| `left_device`, `right_device` | empty | Dual device overrides |
| `left_ifname`, `right_ifname` | empty | Dual EtherCAT interface overrides |
| `headless` | `false` | Run only the Gazebo server |
| `log_level` | `info` | ROS node log level |
| `sdk_log_level` | `info` | GHand SDK console log level |
| `world` | empty | Gazebo world override; empty uses the Gazebo package default |

`profile` and connection overrides apply only to hardware. `world` applies only to Gazebo.
Dual-hand hardware uses the per-hand connection arguments. Each dual launch uses one product.

Preview starts kinematic state publication and planning, with execution disabled.
Mock runs the trajectory controllers using ros2_control GenericSystem so a
`FollowJointTrajectory` goal can be tested without a physical hand or Gazebo.
MoveIt execution starts after the required hardware or simulation, controllers,
and trajectory actions are ready.

Open `log/latest_build` for colcon build logs. Launch prints the runtime log
directory at startup. Set `ROS_LOG_DIR` to choose it, or use the default under
`~/.ros/log`. A hardware configuration error includes the active profile and
connection settings.
Gazebo owns its control manager through its simulation plugin. Physical hardware
uses the bringup control manager. MoveIt never starts a second controller manager.
Only Gazebo uses simulation time.

## Lower-level entries and migration

- `ghand.launch.py`: physical single hand; `hand_side:=left/right`.
- `dual_hand.launch.py`: physical dual hands.
- Both start the state publisher, controllers and optional RViz. Pass `planning:=true` to start MoveIt after activation.
- The lower-level single-hand default profile remains `ethercat_default`;
  the public entry resolves its default profile by hand side.
- `use_mock_hardware` and the old MoveIt `use_sim` argument have been removed.
  These arguments are no longer supported. Choose an explicit scene.
- `moveit.launch.py` and `dual_moveit.launch.py` now default to planning preview.
  Use the public hardware entry with `planning:=true` for hardware planning/execution.
- Stop the current launch before starting another scene.

Hardware connection instructions: [中文](../docs/HARDWARE_CN.md) / [English](../docs/HARDWARE_EN.md).

## Product selection

All hardware, MoveIt and Gazebo launch entry points accept `product_name`.
The default is `ghand5`; choices are discovered from installed
`config/<product_name>/controllers.yaml` files in this package.

```bash
ros2 launch ghand_bringup ghand.launch.py product_name:=ghand5 profile:=rs485_right
ros2 launch ghand_moveit_config dual_moveit.launch.py product_name:=ghand5
```

`profile` (or `left_profile`/`right_profile`) selects a connection independently
of the product. Profiles contain connection settings; the product Xacro supplies
the tactile layout. Dual-hand launches use the same product for both hands.

Keep one hardware plugin, bringup package, description package, Gazebo package
and MoveIt package. A new product needs its own controller/resource data here,
vendor URDF/mesh installation in `ghand_description`, Gazebo configuration/helper,
and SRDF, limits, kinematics and controller mapping in
`ghand_moveit_config/config/<product_name>/`. Review protocol/capability differences
before enabling physical hardware. Do not copy GHand5 values into another product.

GHand5 and Lite1 are available as `ghand5` and `ghandlite1` respectively.
Lite1 has 11 feedback joints and six commanded joints. Its vendor mimic ratios
are preserved; `THUMB_MCP` and the four `*_IP` joints are passive.

```bash
ros2 launch ghand_bringup ghand.launch.py product_name:=ghandlite1 profile:=rs485_right
ros2 launch ghand_bringup ghand.launch.py product_name:=ghandlite1 profile:=ethercat_default ifname:=enx000000000000
ros2 launch ghand_moveit_config moveit.launch.py product_name:=ghandlite1
ros2 launch ghand_gazebo ghand_gazebo.launch.py product_name:=ghandlite1
ros2 launch ghand_moveit_config dual_moveit.launch.py product_name:=ghandlite1
```

The Lite1 protocol baseline is GHand-Cpp-SDK commit `6651e46`: six four-register
command blocks, a two-byte EtherCAT hand header plus six eight-byte commands
(50 bytes), an 11-register feedback header and 11 ten-byte feedback records.
Tactile counts are 38/31/31/31/31, matching the current SDK configuration.
Lite1 supports semi-closed position/speed/current control and CUSTOM selection;
closed-loop, MIT and kp/kd interfaces are unavailable. Position feedback is
motor feedback, not evidence of an independent position sensor.

The vendor assets must include both `ghand5_system` and `ghand_lite1_system`.
For nonstandard asset locations pass `-DGHAND_ASSETS_DIR=<ghand5_system>` and
`-DGHAND_LITE1_ASSETS_DIR=<ghand_lite1_system>` to colcon/CMake. Active Lite1
URDF limits are normalized from `ghand_description/config/ghandlite1.yaml`;
that file records the source and also drives resource and initial-position data.

Hardware compilation, Linux backend startup, MoveIt/Gazebo process startup and
motion/tactile behavior require verification in the target ROS environment.

Description install paths are now `ghand_description/urdf/<product_name>/` and
`ghand_description/meshes/<product_name>/`. Downstream code using direct old paths must
update them; the public launch filenames remain unchanged. Use a fresh install
directory when migrating so obsolete installed configuration files cannot linger.

Runtime development checks use the built ROS workspace: start planning preview,
then Gazebo for control/physics, and finally the selected hardware backend.
Verify controller activation and feedback before sending physical motion commands.

To compare Lite1 metadata with a selected SDK checkout:

```bash
python3 tests/check_lite1_sdk_config.py --sdk-dir /path/to/GHand-Cpp-SDK
```

## SDK adapter architecture

Physical scenes require the separately built `ghand_hardware_interface`,
`ghand_msgs` and `ghand_controllers` packages. Build these with the validated
GHand C++ SDK dynamic library bundled in `ghand_sdk_vendor` using `build.sh`. No local SDK checkout is required. The bringup package
itself can be installed for preview/Gazebo without those optional packages.
SDK owns the device transports and protocols; ROS owns lifecycle, unit conversion
and controller interfaces. Launch files use standard ROS logging and native process-exit shutdown events.
Product choices for planning/simulation are discovered from `ghand_description`.
