# Changelog

All notable changes to the GHand-ROS2 repository will be documented in this
file. See also the per-package `CHANGELOG.rst` files under `src/<package>/`.

## Unreleased

### Changed

* Reorganized hardware packages: moved `ghand_hardware_interface`,
  `hand_controllers`, and `hand_msgs` under `src/ghand_hardware/`. Removed the
  deprecated `hand_bringup` and standalone `src/ghand_hardware_interface/`
  layouts. Launch files and configurations now live in
  `ghand_hardware/ghand_hardware_interface`.

## 1.0.0 (2026-06-30)

### Added

* Initial public release of the GHand-ROS2 control stack.
* Packages: `ghand_description`, `ghand_hardware_interface`, `hand_controllers`,
  `hand_msgs`, `hand_bringup`, `ghand_moveit_config`, `ghand_gazebo`.
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
