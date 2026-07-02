# GHand-ROS2

[![Version](https://img.shields.io/badge/version-v1.0.0-blue.svg)](CHANGELOG.md)
[![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-blue.svg)](https://docs.ros.org/)
[![C++](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![Python](https://img.shields.io/badge/python-3.10%2B-blue.svg)](https://www.python.org/)

[简体中文](README_CN.md)

GHand-ROS2 is a ROS 2 control stack for the GHand five-finger dexterous hand, built on the `ros2_control` framework. It supports Gazebo physical simulation, MoveIt 2 motion planning, and multiple real-hardware communication backends (EtherCAT, CANFD, RS485), enabling algorithm development and visualization verification without hardware.

## 🎬 Demo

![Gazebo Simulation Demo](docs/gazebo_demo.gif)

## ✨ Features

- **Multi-backend hardware drivers**
  - Supports EtherCAT, CANFD, and RS485 communication.
  - Switch backend and device configuration with a single `profile` argument.
  - Unified hardware abstraction layer with consistent interfaces for real and mock hardware.

- **ros2_control integration**
  - Implements `hardware_interface::SystemInterface` lifecycle management.
  - Cyclic read/write of joint commands and states; supports position, velocity, and effort control modes.

- **Custom broadcasters**
  - `joint_status_broadcaster` publishes joint status and error codes.
  - `tactile_broadcaster` publishes tactile resultant force and distributed forces.

- **Simulation and motion planning**
  - Built-in Gazebo Harmonic launch files and ROS 2 Control plugin configuration.
  - Built-in MoveIt 2 configuration for trajectory planning and execution in RViz.

- **Scenario-based launch**
  - `ghand_hardware_interface` handles real / mock hardware launch.
  - `ghand_gazebo` handles physical simulation launch; the two are decoupled.

## 📖 Official Documentation

For detailed technical specifications, API reference, and tutorials, see the [GHand Dexterous Hand Documentation](https://fcnzogxju7xr.feishu.cn/docx/AhZ6ds2iCoguaAxIzBxciYHinNo).

## 📑 Table of Contents

- [Features](#-features)
- [Official Documentation](#-official-documentation)
- [System Requirements](#-system-requirements)
- [Dependencies](#-dependencies)
- [Installation](#-installation)
- [Quick Start](#-quick-start)
- [Directory Structure](#-directory-structure)
- [Open Source and Ecosystem](#-open-source-and-ecosystem)
- [Changelog](#-changelog)
- [Support and Feedback](#-support-and-feedback)
- [License](#-license)

## 💻 System Requirements

### Supported Platforms

| Platform | Requirement |
| -------- | ----------- |
| **Operating System** | Ubuntu 24.04 LTS |
| **ROS 2** | Jazzy Jalisco |
| **Compiler** | GCC 11+ (C++17 support) |
| **Python** | 3.10 or newer |
| **Simulator** | Gazebo Harmonic (required for physical simulation) |

## 🔧 Dependencies

### Required Dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-${ROS_DISTRO}-ros2-control \
  ros-${ROS_DISTRO}-ros2-controllers \
  ros-${ROS_DISTRO}-moveit \
  ros-${ROS_DISTRO}-joint-state-publisher-gui \
  ros-${ROS_DISTRO}-rviz2
```

> Replace `${ROS_DISTRO}` with `jazzy`.

### Optional Hardware Dependencies

Only needed when connecting to real hardware; pure simulation development can skip these.

| Backend | Dependency / Library | Notes |
| ------- | -------------------- | ----- |
| EtherCAT | `libsoem` / SOEM | Build from source; CMake auto-detects |
| RS485 | `libmodbus-dev` | System package or local vendor build |
| CANFD | ZLG CAN SDK / vendor `libusbcanfd.so` | ZLG USB-CANFD library or vendor library |

## 📦 Installation

### 1. Import External Dependencies

The URDF/Xacro and STL meshes for `ghand_description` are provided by external repositories. Import them via `ghand.repos`:

```bash
cd ~/ghand-ros2
vcs import src < ghand.repos
```

### 2. Build

```bash
cd ~/ghand-ros2
colcon build --symlink-install
source install/setup.bash
```

For mock-hardware-only builds, you can also use:

```bash
./build.sh
source install/setup.bash
```

## 🚀 Quick Start

### Launch Simulation (No Hardware Required)

```bash
ros2 launch ghand_hardware_interface ghand.launch.py use_mock_hardware:=true
```

Simulation mode uses `mock_components/GenericSystem` and requires no hardware devices.

### Launch Real Hardware

```bash
ros2 launch ghand_hardware_interface ghand.launch.py profile:=ethercat_default ifname:=enx000000000000
```

> Replace `enx000000000000` with your actual EtherCAT network interface name. Use `ip link` to list available interfaces.

Available communication profiles are located at `src/ghand_hardware/ghand_hardware_interface/config/profiles/` and include `ethercat_default`, `rs485_left`, `rs485_right`, `canfd_usb`, `zqwl_canfd_left`, and `zqwl_canfd_right`.

### Launch MoveIt 2

```bash
ros2 launch ghand_moveit_config moveit.launch.py
```

For EtherCAT real hardware, also provide `ifname`:

```bash
ros2 launch ghand_moveit_config moveit.launch.py use_sim:=false profile:=ethercat_default ifname:=enx000000000000
```

### Launch Gazebo Simulation

```bash
ros2 launch ghand_gazebo ghand_gazebo.launch.py
```

After launch, you can plan trajectories in RViz using the MotionPlanning plugin; trajectories are sent to the `joint_trajectory_controller` running in Gazebo.

## 📁 Directory Structure

```text
GHand-Ros2/
├── src/
│   ├── ghand_hardware/
│   │   ├── ghand_hardware_interface/   # Hardware interface and comm drivers
│   │   ├── hand_controllers/           # Custom broadcasters
│   │   └── hand_msgs/                  # Custom message types
│   ├── ghand_moveit_config/            # MoveIt 2 configuration
│   └── ghand_gazebo/                   # Gazebo physical simulation
├── CHANGELOG.md                        # Version history
├── LICENSE                             # Apache License 2.0
├── README.md                           # This file
└── README_CN.md                        # Chinese README
```

## 🌐 Open Source and Ecosystem Resources

- **GLI Open Source Center**: [GLI SDK GitHub Organization](https://github.com/gli-sdk)
- **Official Documentation**: [GHand Dexterous Hand Documentation](https://fcnzogxju7xr.feishu.cn/docx/AhZ6ds2iCoguaAxIzBxciYHinNo)
- **C++ SDK**: [GHand-Cpp-SDK](https://github.com/gli-sdk/GHand-Cpp-SDK)
- **Python SDK**: [GHand-Python-SDK](https://github.com/gli-sdk/GHand-Python-SDK)

## 📋 Changelog

See [CHANGELOG.md](CHANGELOG.md).

## 📞 Support and Feedback

- 📋 **Technical support**: For project-related questions, please open an `Issue` in this repository.
- 📧 **General inquiries**: [support@glitech.com](mailto:support@glitech.com)

## 📄 License

This project is open-sourced under the [Apache License 2.0](LICENSE).

Copyright © 2026 GHand ROS2 Maintainers.
