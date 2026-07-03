^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ghand_hardware_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Reorganized package under `src/ghand_hardware/ghand_hardware_interface/`.
* Removed hardcoded EtherCAT network interface name from
  `ethercat_default.yaml`. Users must now provide the interface via the
  `ifname` launch argument, e.g.
  `ros2 launch ghand_hardware_interface ghand.launch.py profile:=ethercat_default ifname:=enx000000000000`.
* Contributors: GHand ROS2 Maintainers

1.0.0 (2026-06-30)
------------------
* Initial release of the GHand-ROS2 stack.
* Public headers moved from `include/ghand_hardware/` to
  `include/ghand_hardware_interface/` and renamed from `.h` to `.hpp`.
  Update `#include "ghand_hardware/ghand_hardware.h"` to
  `#include "ghand_hardware_interface/ghand_hardware.hpp"`.
* Communication headers moved from `include/communication/` to
  `include/ghand_hardware_interface/communication/`.
  Update includes such as `#include "communication/communication.hpp"` to
  `#include "ghand_hardware_interface/communication/communication.hpp"`.
* Contributors: GHand ROS2 Maintainers
