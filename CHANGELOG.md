# Changelog

All notable changes to the GHand-ROS2 repository will be documented in this
file. See also the per-package `CHANGELOG.rst` files.

## Unreleased

### Changed

* Removed the top-level `src/` directory; all packages now reside directly in
  the repository root. Updated installation instructions in `README.md` and
  `README_CN.md` to use a standard ROS 2 workspace layout: clone the
  repository into `<workspace>/src` and import dependencies with
  `vcs import . < ghand-ros2/ghand.repos` from the workspace src directory.
* Reorganized hardware packages: moved `ghand_hardware_interface`,
  `ghand_controllers`, and `ghand_msgs` under `ghand_hardware/`. Removed the
  deprecated `hand_bringup` and standalone `ghand_hardware_interface/`
  layouts. Launch files and configurations now live in
  `ghand_hardware/ghand_hardware_interface`.
* EtherCAT default profile (`ethercat_default.yaml`) no longer contains a
  hardcoded network interface name. Users must now provide the interface via
  the `ifname` launch argument, e.g.
  `ros2 launch ghand_hardware_interface ghand.launch.py profile:=ethercat_default ifname:=enx000000000000`.

## 1.0.0 (2026-06-30)

### Added

* Initial public release of the GHand-ROS2 control stack.
* Packages: `ghand_description`, `ghand_hardware_interface`, `ghand_controllers`,
  `ghand_msgs`, `hand_bringup`, `ghand_moveit_config`, `ghand_gazebo`.
* `ros2_control` integration with optional EtherCAT, RS485, and CANFD backends.
* Gazebo Harmonic simulation support.
* MoveIt 2 motion planning configuration.
* Repository onboarding documentation (README, CONTRIBUTING, CODE_OF_CONDUCT,
  SECURITY, issue/PR templates).
* Build helper `build.sh` and GitHub Actions CI workflow.

### Changed

* Public C++ headers moved from `include/ghand_hardware/` to
  `include/ghand_hardware_interface/` and renamed from `.h` to `.hpp`. Update
  `#include "ghand_hardware/ghand_hardware.h"` to
  `#include "ghand_hardware_interface/ghand_hardware.hpp"`.
* Communication headers moved from `include/communication/` to
  `include/ghand_hardware_interface/communication/`. Update includes such as
  `#include "communication/communication.hpp"` to
  `#include "ghand_hardware_interface/communication/communication.hpp"`.

### Notes

* This is the first public release. Internal testing is maintained outside the
  public repository per project policy.
