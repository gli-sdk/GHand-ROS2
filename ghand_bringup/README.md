# ghand_bringup

Launch and runtime configuration for single and dual GHands.

```bash
ros2 launch ghand_bringup ghand.launch.py use_mock_hardware:=true
ros2 launch ghand_bringup ghand.launch.py profile:=ethercat_default ifname:=enx000000000000
ros2 launch ghand_bringup dual_hand.launch.py use_mock_hardware:=true
```

`launch/` starts the state publisher, controller manager, controllers and RViz.
`config/ghand5/` contains GHand5 controller YAML, RViz settings and joint/sensor
resource Xacros. `config/common/` contains the shared hardware plugin and
connection macros. The top-level description Xacros select the product directory.
Existing launch arguments and defaults are retained.

Launch files previously provided by `ghand_hardware_interface` are now launched
from `ghand_bringup`. Communication profiles are under `config/profiles/`.
Gazebo and MoveIt keep their own launch files and specialized configuration.

## Product selection

All six hardware, MoveIt and Gazebo launch entry points accept `product_name`.
The default is `ghand5`; choices are discovered from installed
`config/<product_name>/controllers.yaml` files in this package.

```bash
ros2 launch ghand_bringup ghand.launch.py product_name:=ghand5 use_mock_hardware:=true
ros2 launch ghand_moveit_config dual_moveit.launch.py product_name:=ghand5 use_sim:=true
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
ros2 launch ghand_bringup ghand.launch.py product_name:=ghandlite1 use_mock_hardware:=true
ros2 launch ghand_bringup ghand.launch.py product_name:=ghandlite1 profile:=ethercat_default ifname:=enx000000000000
ros2 launch ghand_moveit_config moveit.launch.py product_name:=ghandlite1 use_sim:=true
ros2 launch ghand_gazebo ghand_gazebo.launch.py product_name:=ghandlite1
ros2 launch ghand_moveit_config dual_moveit.launch.py product_name:=ghandlite1 use_sim:=true
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

Offline configuration checks (Python, PyYAML and xacro required):

```bash
python3 -m unittest discover -s tests -v
```

The xacro checks use fixture geometry. They do not replace a ROS 2 build,
Gazebo/MoveIt launch checks or device tests.

To compare Lite1 metadata with a selected SDK checkout:

```bash
python3 tests/check_lite1_sdk_config.py --sdk-dir /path/to/GHand-Cpp-SDK
```
