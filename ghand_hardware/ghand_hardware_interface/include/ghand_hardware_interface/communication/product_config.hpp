// Copyright (c) 2026 GHand ROS2 Maintainers
// SPDX-License-Identifier: Apache-2.0
#ifndef GHAND_HARDWARE_INTERFACE_COMMUNICATION_PRODUCT_CONFIG_HPP_
#define GHAND_HARDWARE_INTERFACE_COMMUNICATION_PRODUCT_CONFIG_HPP_

#include "ghand_hardware_interface/communication/communication.hpp"

namespace ghand_comm {

// Source: GHand-Cpp-SDK 6651e46, config/ghand*.json and
// src/internal/product_config_loader.cc; L1 V3.1 command/feedback layout.
struct ProductConfig {
  std::string name;
  std::vector<JointId> joints;
  std::vector<JointId> active_joints;
  uint16_t hand_register_count;
  uint16_t joint_input_register;
  uint16_t command_register_count;
  bool lite1() const { return name == "ghandlite1"; }
  size_t rxpdo_size() const { return 2 + active_joints.size() * command_register_count * 2; }
  std::map<JointId, uint16_t> holding_registers() const;
};

// Unknown names never fall back to GHand5.
const ProductConfig* find_product_config(const std::string& name);

}  // namespace ghand_comm
#endif
