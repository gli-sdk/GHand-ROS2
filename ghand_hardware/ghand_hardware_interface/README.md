# ghand_hardware_interface

A thin `ros2_control::SystemInterface` adapter for GHand C++ SDK 3.1 or newer
in the 3.x API family. Device communication, discovery, protocol encoding,
product capabilities and device commands are owned by the SDK.
This package does not implement RS485, CANFD or EtherCAT protocols.

## Build

This project includes the SDK-generated dynamic library, matching public headers,
product JSON, CMake exports and license notices under `ghand_sdk_vendor/sdk/`.
Neither the build nor runtime accesses a developer's SDK source checkout.

From the workspace root, after sourcing ROS 2 Jazzy:

```bash
./src/ghand-ros2/build.sh
source install/setup.bash
```

The adapter finds the installed `ghand_sdk_vendor` package and links its
`GHand::ghand` target (`libghand.so`). ROS does not compile SDK sources. Joint and
active-joint validation tables are generated from this bundled SDK's JSON.
The vendor package environment adds the installed library/config paths.
Use `build.sh --simulation` for preview/Gazebo without building or installing SDK.

SDK upgrades replace the complete matching SDK artifact set in
`ghand_sdk_vendor/sdk/`; rebuild the vendor package and hardware adapter afterward.
The included artifact is Linux x86_64, SDK 3.1.0. Its binary, headers and JSON
must be updated together. See `ghand_sdk_vendor/SDK_MANIFEST.json` for hashes.

## Runtime

```bash
ros2 launch ghand_bringup start.launch.py backend:=hardware \
  product_name:=ghand5 hand:=right planning:=false \
  profile:=rs485_right device:=/dev/ttyUSB0
ros2 run ghand_hardware_interface ghand_joint_controller_gui
```

The plugin remains `ghand_hardware_interface/GHandHardwareInterface`. Existing
position trajectory actions, joint names and custom feedback topics are retained.
The SDK session accepts `rs485`, `canfd`, `ethercat`; `zqwl_canfd` is an alias
for SDK CANFD. The old independent `zlg_canfd` implementation is removed.

Connection parameters: `product_name`, `device` (serial/CAN adapter), `ifname`
(EtherCAT), `node_id`, `baud_rate`, `data_rate`, and `control_mode`.
`hand_side` or `slave_index` selects a hand when one SDK session discovers
multiple hands. Existing dual-hand launches use separate adapters/sessions;
sharing one bus between separate hardware plugins is not supported.
RS485 uses SDK-supported baud rates and 8N1; CANFD bitrates must match a typed
SDK bit-timing profile. Low-level polling, serial adapter and response-timeout
settings are managed by SDK defaults, rather than reimplemented in ROS.
`read_timeout_ms` is the ROS joint-feedback freshness deadline (default 500 ms).

## ROS / device boundary

- Position: ROS radians, SDK degrees, mapped by explicit SDK joint IDs.
- `velocity_percent` and `current_percent`: device percentages, not rad/s or Nm.
- Commands additionally expose the declared `control_mode`, `loop_type`,
  `control_state`, `kp`, `kd` interfaces. Device capability restrictions remain
  enforced by SDK; Lite1-specific ROS interfaces remain restricted.
- Unwritten commands remain NaN. No motion is sent while controllers load.
- Hand state and tactile data come from SDK callbacks. Missing data is not a
  valid new measurement; joint feedback age uses a monotonic clock.

Configure connects through SDK. Activate requires fresh complete feedback and
clears previous commands. Deactivate invokes SDK `Stop()` and keeps the session
available for reactivation; cleanup disconnects. Error/shutdown stops and
releases the connection. SDK errors propagate to ROS, including SDK error codes
and messages. `read()` copies the callback cache; `write()` calls `MoveJoints()`.
Write latency therefore follows the SDK API and selected transport.

## Verification scope

SDK linking and the ROS adapter build have been checked. Device movement,
controller execution and simulation behavior are to be verified by the user.
No device protocol tests are maintained in this ROS package.
