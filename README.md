# GHand-ROS2

A ROS 2 control stack for the GHand five-finger dexterous hand, built on the `ros2_control` framework. The project separates low-level hardware abstraction from upper-layer application logic, supports simulation-first development, and integrates with Gazebo, MoveIt 2, Isaac Sim, and other toolchains.

---

## Tech Stack

| Item | Version / Notes |
|------|-----------------|
| OS | Ubuntu 22.04 LTS or newer |
| ROS 2 | Humble Hawksbill / Jazzy |
| Languages | C++17, Python 3.10+ |
| Build tool | colcon (CMake for C++) |
| Simulator | Gazebo Harmonic |
| Motion planning | MoveIt 2 |
| EtherCAT master | SOEM (`libsoem`) |
| CANFD master | ZLG CAN (`libzlgcan`) |

---

## Architecture

```text
┌─────────────────────────────────────┐
│  Application: MoveIt 2 / RViz /     │
│  custom nodes                       │
├─────────────────────────────────────┤
│  Controllers: joint_trajectory_     │
│  controller, joint_state_broadcaster│
│  joint_status_broadcaster           │
│  tactile_broadcaster                │
├─────────────────────────────────────┤
│  ros2_control: GHand_hardware       │
│  (on_init / on_configure / read /   │
│   write)                            │
├─────────────────────────────────────┤
│  Hardware drivers: EtherCAT /       │
│  CANFD / RS485                      │
└─────────────────────────────────────┘
```

- **Hardware layer**: abstracts EtherCAT, CANFD, and RS485 through the unified `communication.hpp` interface.
- **ros2_control layer**: `GHand_hardware` implements `hardware_interface::SystemInterface` and manages the hardware lifecycle and cyclic read/write.
- **Controller layer**: custom broadcasters publish `/joint_status` and `/tactile_data`.
- **Launch layer**: `ghand_hardware_interface` loads the robot description, controller manager, and controllers.

---

## Package Structure

| Package | Description |
|---------|-------------|
| `ghand_description` | Hand URDF/Xacro description and STL meshes |
| `ghand_hardware_interface` | Hardware component (launch + config), implements `hardware_interface::SystemInterface` |
| `hand_controllers` | Custom broadcasters: `joint_status_broadcaster`, `tactile_broadcaster` |
| `hand_msgs` | Custom message types: `JointStatus`, `TactileData` |
| `ghand_moveit_config` | MoveIt 2 motion-planning configuration |
| `ghand_gazebo` | Gazebo physical simulation launch and configuration |

---

## Quick Start

### 1. Environment Preparation

- Install ROS 2 (Humble or Jazzy) and source the setup files.
- Install dependencies (Jazzy example):

```bash
sudo apt update
sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-moveit ros-jazzy-joint-state-publisher-gui ros-jazzy-rviz2
```

- For real hardware, make sure `libsoem` and/or `libzlgcan` is installed.

#### Optional: Install Real-Hardware Dependencies

The following are only needed when connecting to real hardware; pure simulation development can skip them.

**EtherCAT (SOEM)**

ROS 2 Jazzy does not provide a `ros-jazzy-soem` apt package, so SOEM must be built from source:

```bash
cd ~
git clone https://github.com/OpenEtherCATsociety/SOEM.git
cd SOEM
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/home/pq/SOEM/build/install -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..
make -j$(nproc)
make install
```

> Notes:
> - `CMAKE_POSITION_INDEPENDENT_CODE=ON` is required because `libsoem.a` is a static library that will be linked into this project's shared library.
> - This project's `CMakeLists.txt` automatically detects `soemConfig.cmake` under `~/SOEM/build/install/cmake`; no manual environment variables are needed.

**RS485 (libmodbus)**

```bash
sudo apt update
sudo apt install -y libmodbus-dev
```

**CANFD (ZLG CAN)**

ZLG CAN is a proprietary SDK for ZLG CAN interfaces. Obtain it from the official channel:

1. Download the Linux driver and SDK for your device (usually `libzlgcan.so` and header `zlgcan.h`).
2. Place the header in `/usr/local/include/` and the library in `/usr/local/lib/`:

```bash
sudo cp zlgcan.h /usr/local/include/
sudo cp libzlgcan.so /usr/local/lib/
sudo ldconfig
```

3. Verify CMake can find it:

```bash
cd ~/GHand-Ros2
colcon build --packages-select ghand_hardware_interface
```

Expected output: `Found ZLG CAN library; CANFD backend enabled` (or no longer disabled).

### 2. Build

The URDF/Xacro and STL meshes for `ghand_description` are provided by an external repository. Import them before building:

```bash
cd ~/GHand-Ros2
vcs import src < ghand.repos
```

Then build the workspace:

```bash
cd ~/GHand-Ros2  # or your repository directory
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch Simulation (No Hardware Required)

```bash
ros2 launch ghand_hardware_interface ghand.launch.py use_mock_hardware:=true
```

Simulation mode uses `mock_components/GenericSystem` as the backend and requires no EtherCAT/CANFD/RS485 devices. It is suitable for development, visualization, and CI.

### 4. Launch Real Hardware

Real-hardware mode loads `GHand_hardware/GHandHardwareInterface` and selects the backend and device parameters via a **communication profile**. Before launching, confirm:

- The correct profile is selected for your hardware;
- EtherCAT/RS485/CANFD interfaces are connected and configured;
- The current user has read/write access to serial/CAN devices (RS485 requires the `dialout` group).

```bash
ros2 launch ghand_hardware_interface ghand.launch.py profile:=ethercat_default ifname:=enx000000000000
```

> Replace `enx000000000000` with your actual EtherCAT network interface name. Use `ip link` to list available interfaces.

#### 4.1 Communication Profiles

All backend parameters are placed under `src/ghand_hardware/ghand_hardware_interface/config/profiles/`. A single `profile` argument switches the backend:

| Profile | Backend | Notes |
|---------|---------|-------|
| `ethercat_default` | EtherCAT | Specify interface via `ifname` |
| `rs485_left` | RS485 | Left hand, slave ID `49` (`0x31`), device `/dev/ttyUSB0` |
| `rs485_right` | RS485 | Right hand, slave ID `50` (`0x32`), device `/dev/ttyUSB0` |
| `canfd_usb` | CANFD (ZLG USB-CANFD) | Device `ZCAN_USBCANFD_100U:0:0`, ID `49` |
| `zqwl_canfd_left` | ZQWL USB-CANFD | Left hand, serial `/dev/ttyACM0`, CAN ID `49` |
| `zqwl_canfd_right` | ZQWL USB-CANFD | Right hand, serial `/dev/ttyACM0`, CAN ID `50` |

Unified parameter names (used by both profiles and the low-level drivers):

| Parameter | Description |
|-----------|-------------|
| `communication_type` | Backend type: `ethercat` / `rs485` / `canfd` / `zqwl_canfd` |
| `device` | Device path or identifier |
| `baud_rate` | RS485 serial baud rate / CANFD arbitration-phase baud rate |
| `data_rate` | CANFD data-phase baud rate |
| `node_id` | Slave ID / CAN device ID (formerly `slave_id` / `device_id`) |
| `ifname` | EtherCAT network interface name |

For backward compatibility, the following legacy arguments can still be used as overrides (higher priority than the profile):

| Legacy argument | Maps to |
|-----------------|---------|
| `communication_type` | `communication_type` |
| `device` | `device` |
| `baud_rate` | `baud_rate` |
| `dbit_rate` | `data_rate` |
| `slave_id` | `node_id` |
| `ifname` | `ifname` |

Example: use a profile but temporarily override the device:

```bash
ros2 launch ghand_hardware_interface ghand.launch.py profile:=rs485_left device:=/dev/ttyUSB1
```

#### 4.2 Build Check

```bash
cd ~/GHand-Ros2
# If libmodbus is not system-installed, build it locally (required for RS485)
export PKG_CONFIG_PATH=/home/pq/GHand-Ros2/vendor/libmodbus-install/lib/pkgconfig:$PKG_CONFIG_PATH
colcon build --packages-select ghand_hardware_interface --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Expected results:
- `ghand_hardware_interface` builds successfully with no errors or warnings.
- If `libmodbus` is found, CMake no longer prints `RS485 backend will be disabled at runtime`.

#### 4.3 RS485 Verification

##### Preparation

1. Install or build `libmodbus`:

```bash
# Option 1: system package (recommended)
sudo apt update
sudo apt install -y libmodbus-dev

# Option 2: local source build
export PKG_CONFIG_PATH=/home/pq/GHand-Ros2/vendor/libmodbus-install/lib/pkgconfig:$PKG_CONFIG_PATH
```

2. Add the current user to the `dialout` group and log in again:

```bash
sudo usermod -aG dialout $USER
# Re-login or run newgrp dialout
groups  # confirm dialout is present
```

3. Confirm device nodes:

```bash
ls -l /dev/ttyUSB*
```

##### Launch RS485

```bash
ros2 launch ghand_hardware_interface ghand.launch.py profile:=rs485_left
```

> `node_id` must match the Modbus slave ID configured on the hand driver board; `rs485_left` is `49` (`0x31`) and `rs485_right` is `50` (`0x32`).

To temporarily override the device or slave ID:

```bash
ros2 launch ghand_hardware_interface ghand.launch.py profile:=rs485_left device:=/dev/ttyUSB1 slave_id:=50
```

##### Verification

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic echo /joint_states
```

Expected results:
- Log shows `Successfully configured!` / `Successfully activated!`.
- `/joint_states` updates continuously, with positions/velocities matching the real fingers.
- After sending position commands, the fingers move physically with no serial timeouts or CRC errors.

##### Send Position Commands

```bash
# Use position_controller directly (must switch controllers first)
python3 /home/pq/send_hand_position_direct.py
```

##### Performance Notes

Early implementations sent one Modbus frame per joint in `Rs485Driver::write()`, causing severe 100 Hz control-cycle overruns (write took ~88 ms). The current version uses **batch writes to contiguous holding registers** (0x0011~0x002A, 26 registers total), reducing write time to below ~10 ms and meeting real-time control requirements.

#### 4.4 EtherCAT Verification

Launch:

```bash
ros2 launch ghand_hardware_interface ghand.launch.py profile:=ethercat_default ifname:=enx000000000000
```

To specify a different EtherCAT interface:

```bash
ros2 launch ghand_hardware_interface ghand.launch.py profile:=ethercat_default ifname:=eth0
```

Verification:

```bash
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states
```

Expected results:
- Log shows `Initialized ethercat communication backend.`
- PDO data refreshes normally; `/joint_states` updates synchronously as the fingers move.

#### 4.5 CANFD Verification

The CANFD backend uses the ZLG VCI library under `vendor/usbcanfd_libusb_x64_1.0.12_251029/`:

```text
vendor/usbcanfd_libusb_x64_1.0.12_251029/libusbcanfd.so
vendor/usbcanfd_libusb_x64_1.0.12_251029/libzuds.so
vendor/dist/include/zlgcan/zcan.h
```

No additional installation is required; the vendor libraries are linked automatically at build time.

##### Build

```bash
cd ~/GHand-Ros2
colcon build --packages-select ghand_hardware_interface ghand_description hand_controllers hand_msgs --symlink-install
source install/setup.bash
```

Expected: `ghand_hardware_interface` builds successfully and CMake no longer prints `ZLG CAN library not found`.

##### Launch CANFD

`device` format is `device_type:device_index:channel_index`, e.g. `ZCAN_USBCANFD_100U:0:0`. `baud_rate` is the CANFD arbitration-phase baud rate and `data_rate` is the data-phase baud rate.

```bash
ros2 launch ghand_hardware_interface ghand.launch.py profile:=canfd_usb
```

To override device or baud rate:

```bash
ros2 launch ghand_hardware_interface ghand.launch.py \
  profile:=canfd_usb \
  device:=ZCAN_USBCANFD_100U:0:0 \
  baud_rate:=1000000 \
  dbit_rate:=5000000 \
  slave_id:=49
```

> Legacy arguments `dbit_rate` and `slave_id` still map to unified parameters `data_rate` and `node_id`.
> If the device CAN clock is not 60 MHz, modify `can_clk` in the profile or override with `can_clk:=<frequency>`.

##### Verification

The CANFD backend is mainly used to read tactile data:

```bash
ros2 topic echo /tactile_data
```

Expected results:
- Log shows `Successfully configured!` / `Successfully activated!`.
- When touching the sensor, `resultant_force` and `distributed_forces` change significantly.

#### 4.6 ZQWL USB-CANFD Verification

The ZQWL backend uses USB serial-to-CANFD with the `zqwl_canfd_left` / `zqwl_canfd_right` profiles:

```bash
ros2 launch ghand_hardware_interface ghand.launch.py profile:=zqwl_canfd_left
```

To override the serial device or CAN ID:

```bash
ros2 launch ghand_hardware_interface ghand.launch.py \
  profile:=zqwl_canfd_left \
  device:=/dev/ttyACM1 \
  slave_id:=50
```

> Legacy argument `slave_id` maps to unified parameter `node_id`.

#### 4.7 Error Behavior Without Hardware

Disconnect the device and launch:

```bash
ros2 launch ghand_hardware_interface ghand.launch.py profile:=rs485_left
```

Expected results:
- Log clearly prints `Failed to initialize rs485 communication backend.`.
- ⚠️ In the current version, `ros2_control_node` throws `std::runtime_error` and aborts on configure failure. A future improvement is to gracefully degrade to `on_error()` instead of crashing.

### 5. Launch MoveIt 2

```bash
source /home/pq/GHand-Ros2/install/setup.bash
ros2 launch ghand_moveit_config moveit.launch.py
```

By default mock hardware is used. To connect to real hardware, specify the `profile` argument:

```bash
ros2 launch ghand_moveit_config moveit.launch.py use_sim:=false profile:=rs485_left
```

For EtherCAT real hardware, also provide `ifname`:

```bash
ros2 launch ghand_moveit_config moveit.launch.py use_sim:=false profile:=ethercat_default ifname:=enx000000000000
```

### 6. Launch Gazebo Simulation

Make sure Gazebo dependencies are installed:

```bash
sudo apt update
sudo apt install -y ros-jazzy-gz-ros2-control ros-jazzy-ros-gz-sim ros-jazzy-ros-gz
```

Launch the Gazebo physical simulation (includes MoveIt):

```bash
source /home/pq/GHand-Ros2/install/setup.bash
ros2 launch ghand_gazebo ghand_gazebo.launch.py
```

After launch, Gazebo loads `empty.world` and spawns the GHand model. In RViz, you can plan trajectories with the MotionPlanning plugin; trajectories are sent to the `joint_trajectory_controller` running in Gazebo.

---

## Controlling the Hand

After launching `ghand_hardware_interface`, you can send joint-trajectory commands via `rqt_joint_trajectory_controller` or example scripts to quickly verify controller behavior.

### Using `rqt_joint_trajectory_controller`

> Note: The GHand URDF contains 18 joints, but the EtherCAT slave only drives 13 active joints (excluding DIP passive joints). `rqt_joint_trajectory_controller` generates 18 sliders based on the URDF. Sending 18-joint trajectories directly to `joint_trajectory_controller` will fail because the passive joints cannot track, so the real hand will not move.
>
> Therefore:
> - **Mock mode** (`use_mock_hardware:=true`) can use rqt directly;
> - **Real hardware**: only control the 13 active joints, or write a custom node that converts rqt's 18-joint topic into a 13-joint action goal (the original `rqt_jtc_bridge.py` was removed during package reorganization).

#### Mock Mode / Visualization Debug

```bash
ros2 launch ghand_hardware_interface ghand.launch.py use_mock_hardware:=true
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

After opening the UI:

1. Select the controller manager namespace in the top-left (usually empty, i.e. default namespace).
2. Select `joint_trajectory_controller` from the controller dropdown.
3. Drag the joint sliders and click **Enable** to send position commands to the hand.

#### Real Hardware

```bash
source /home/pq/GHand-Ros2/install/setup.bash
ros2 launch ghand_hardware_interface ghand.launch.py use_mock_hardware:=false
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

Then follow the same steps as in mock mode.

> Note: In Jazzy, `rqt_joint_trajectory_controller` sometimes does not correctly sync the **Enable** button state. If dragging sliders does not move the hand, try clicking **Disable**, then **Enable**, and drag the sliders again.

### Using Example Scripts

> Note: The `send_hand_position.py` example script in `ghand_hardware_interface` was removed during package reorganization. If you need it, re-implement it against the current controller interface or restore it from Git history.

---

## Common Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Joint positions, velocities, efforts |
| `/joint_status` | `hand_msgs/JointStatus` | Joint status and error codes |
| `/tactile_data` | `hand_msgs/TactileData` | Tactile resultant force and distributed forces |
| `/controller_manager/robot_description` | `std_msgs/String` | Robot URDF description |

---

## Configuration Notes

- `src/ghand_hardware/ghand_hardware_interface/config/profiles/`: communication backend profiles (EtherCAT/RS485/CANFD/ZQWL CANFD).
- `src/ghand_hardware/ghand_hardware_interface/config/ros2_control.yaml`: hardware component parameters, communication interface configuration, joint list.
- `src/ghand_hardware/ghand_hardware_interface/config/controllers.yaml`: controller manager parameters, controller types, and PID parameters.
- `src/ghand_hardware/ghand_hardware_interface/config/ghand.rviz`: RViz visualization configuration.
- `src/ghand_moveit_config/config/`: MoveIt kinematics, planner, and joint-limit configuration.

---

## Development Conventions

- C++ headers use `.hpp` and source files use `.cpp`.
- Plugin description files use `.xml` (e.g. `hand_hardware_interface.xml`).
- Message definitions live in `msg/*.msg`.
- Launch files use Python format (`.launch.py`).
- Configuration files are placed in each package's `config/` directory.

---

## Directory Structure

```text
GHand-Ros2/                              # ROS 2 workspace
├── src/
│   ├── ghand_description/               # URDF/Xacro and model files
│   ├── ghand_hardware/                  # Hardware-related packages
│   │   ├── ghand_hardware_interface/    # Hardware interface and comm drivers
│   │   ├── hand_controllers/            # Custom broadcasters
│   │   └── hand_msgs/                   # Custom messages
│   ├── ghand_moveit_config/             # MoveIt 2 configuration
│   └── ghand_gazebo/                    # Gazebo physical simulation
├── openspec/                            # Specification-driven development docs
├── ros.md                               # Project design document
├── ros2_control架构图.drawio.png        # Architecture diagram
└── README.md                            # This file
```

---

## License

MIT
