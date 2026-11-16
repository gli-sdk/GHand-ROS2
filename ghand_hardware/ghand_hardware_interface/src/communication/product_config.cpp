// Copyright (c) 2026 GHand ROS2 Maintainers
// SPDX-License-Identifier: Apache-2.0
#include "ghand_hardware_interface/communication/product_config.hpp"

namespace ghand_comm {

const ProductConfig* find_product_config(const std::string& name) {
  static const ProductConfig g5 = {
      "ghand5",
      {JointId::THUMB_DIP, JointId::THUMB_MCP, JointId::THUMB_TMC_FE,
       JointId::THUMB_TMC_AA, JointId::THUMB_TMC_PS, JointId::INDEX_DIP,
       JointId::INDEX_PIP, JointId::INDEX_MCP, JointId::INDEX_MCP_AA,
       JointId::MIDDLE_DIP, JointId::MIDDLE_PIP, JointId::MIDDLE_MCP,
       JointId::RING_DIP, JointId::RING_PIP, JointId::RING_MCP,
       JointId::LITTLE_DIP, JointId::LITTLE_PIP, JointId::LITTLE_MCP},
      {JointId::THUMB_MCP, JointId::THUMB_TMC_FE, JointId::THUMB_TMC_AA,
       JointId::THUMB_TMC_PS, JointId::INDEX_PIP, JointId::INDEX_MCP,
       JointId::INDEX_MCP_AA, JointId::MIDDLE_PIP, JointId::MIDDLE_MCP,
       JointId::RING_PIP, JointId::RING_MCP, JointId::LITTLE_PIP,
       JointId::LITTLE_MCP}, 14, 0x105F, 6};
  static const ProductConfig l1 = {
      "ghandlite1",
      {JointId::THUMB_MCP, JointId::THUMB_TMC_FE, JointId::THUMB_TMC_AA,
       JointId::INDEX_PIP, JointId::INDEX_MCP, JointId::MIDDLE_PIP,
       JointId::MIDDLE_MCP, JointId::RING_PIP, JointId::RING_MCP,
       JointId::LITTLE_PIP, JointId::LITTLE_MCP},
      {JointId::THUMB_TMC_FE, JointId::THUMB_TMC_AA, JointId::INDEX_MCP,
       JointId::MIDDLE_MCP, JointId::RING_MCP, JointId::LITTLE_MCP},
      11, 0x105C, 4};
  if (name == g5.name) return &g5;
  if (name == l1.name) return &l1;
  return nullptr;
}

std::map<JointId, uint16_t> ProductConfig::holding_registers() const {
  std::map<JointId, uint16_t> result;
  for (size_t i = 0; i < active_joints.size(); ++i)
    result[active_joints[i]] = static_cast<uint16_t>(0x0261 + i * command_register_count);
  return result;
}

}  // namespace ghand_comm
