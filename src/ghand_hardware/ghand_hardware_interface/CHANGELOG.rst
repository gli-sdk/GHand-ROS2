^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ghand_hardware_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
