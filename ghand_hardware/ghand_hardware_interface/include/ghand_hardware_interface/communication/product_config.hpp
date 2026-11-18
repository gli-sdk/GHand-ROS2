// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <string>
#include <vector>
#include "ghand_hardware_interface/communication/sdk_types.hpp"
namespace ghand_comm {
// Generated from the installed SDK's public product JSON, not protocol layouts.
struct ProductConfig {
  std::string name;
  std::vector<JointId> joints;
  std::vector<JointId> active_joints;
  bool lite1() const { return name == "ghandlite1"; }
};
const ProductConfig* find_product_config(const std::string& name);
}
