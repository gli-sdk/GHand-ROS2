// Copyright (c) 2026 GHand ROS2 Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "ghand_hardware_interface/communication/communication.hpp"

#include <array>
#include <map>
#include <memory>
#include <string>

#include "ghand_hardware_interface/communication/canfd_driver.hpp"
#include "ghand_hardware_interface/communication/ethercat_driver.hpp"
#include "ghand_hardware_interface/communication/rs485_driver.hpp"
#include "ghand_hardware_interface/communication/zqwl_canfd_driver.hpp"

namespace ghand_comm {

namespace {

constexpr size_t kJointCount = static_cast<size_t>(JointId::NUM_JOINTS);

const std::array<std::string, kJointCount + 1> kJointIdNames = {
    "Thumb_IP",  "Thumb_MCP",  "Thumb_TMC_FE", "Thumb_TMC_AA", "Thumb_TMC_PS",
    "FF_DIP",  "FF_PIP",  "FF_MCP",    "FF_MCP_AA", "MF_DIP",
    "MF_PIP", "MF_MCP", "RF_DIP",     "RF_PIP",     "RF_MCP",
    "LF_DIP", "LF_PIP", "LF_MCP",   "UNKNOWN",
};

const std::map<JointId, uint16_t> kHoldingRegMap = {
    {JointId::THUMB_MCP, 0x0011},    {JointId::THUMB_TMC_FE, 0x0013},
    {JointId::THUMB_TMC_AA, 0x0015}, {JointId::THUMB_TMC_PS, 0x0017},
    {JointId::INDEX_PIP, 0x0019},    {JointId::INDEX_MCP, 0x001B},
    {JointId::INDEX_MCP_AA, 0x001D}, {JointId::MIDDLE_PIP, 0x001F},
    {JointId::MIDDLE_MCP, 0x0021},   {JointId::RING_PIP, 0x0023},
    {JointId::RING_MCP, 0x0025},     {JointId::LITTLE_PIP, 0x0027},
    {JointId::LITTLE_MCP, 0x0029},
};

}  // namespace

std::optional<JointId> joint_name_to_id(const std::string& name) {
  // Dual-hand descriptions prefix ros2_control resources to keep the two
  // systems collision-free. The wire protocol continues to use the canonical
  // unprefixed GHand joint IDs.
  std::string canonical_name = name;
  for (const std::string prefix : {"left_", "right_"}) {
    if (canonical_name.rfind(prefix, 0) == 0) {
      canonical_name.erase(0, prefix.size());
      break;
    }
  }

  if (canonical_name == "Thumb_TMC_PS") return JointId::THUMB_TMC_PS;
  if (canonical_name == "Thumb_TMC_AA") return JointId::THUMB_TMC_AA;
  if (canonical_name == "Thumb_TMC_FE") return JointId::THUMB_TMC_FE;
  if (canonical_name == "Thumb_MCP") return JointId::THUMB_MCP;
  if (canonical_name == "Thumb_IP") return JointId::THUMB_DIP;

  if (canonical_name == "FF_MCP_AA") return JointId::INDEX_MCP_AA;
  if (canonical_name == "FF_MCP") return JointId::INDEX_MCP;
  if (canonical_name == "FF_PIP") return JointId::INDEX_PIP;
  if (canonical_name == "FF_DIP") return JointId::INDEX_DIP;

  if (canonical_name == "MF_MCP") return JointId::MIDDLE_MCP;
  if (canonical_name == "MF_PIP") return JointId::MIDDLE_PIP;
  if (canonical_name == "MF_DIP") return JointId::MIDDLE_DIP;

  if (canonical_name == "RF_MCP") return JointId::RING_MCP;
  if (canonical_name == "RF_PIP") return JointId::RING_PIP;
  if (canonical_name == "RF_DIP") return JointId::RING_DIP;

  if (canonical_name == "LF_MCP") return JointId::LITTLE_MCP;
  if (canonical_name == "LF_PIP") return JointId::LITTLE_PIP;
  if (canonical_name == "LF_DIP") return JointId::LITTLE_DIP;

  return std::nullopt;
}

const std::string& joint_id_to_name(JointId id) {
  const size_t index = static_cast<size_t>(id);
  if (index < kJointIdNames.size()) {
    return kJointIdNames[index];
  }
  return kJointIdNames.back();
}

const std::map<JointId, uint16_t>& holding_register_map() {
  return kHoldingRegMap;
}

std::unique_ptr<CommunicationBase> create_communication(
    const std::string& type) {
  if (type == "ethercat") {
    return std::make_unique<EthercatDriver>();
  }
  if (type == "rs485") {
    return std::make_unique<Rs485Driver>();
  }
  if (type == "canfd") {
    // The canonical CANFD backend follows GHand-Cpp-SDK: a ZQWL CDC serial
    // adapter carrying Modbus-over-CANFD frames.  Keep the old ZLG/VCI
    // implementation available under an explicit legacy name only.
    return std::make_unique<ZqwlCanfdDriver>();
  }
  if (type == "zqwl_canfd") {
    return std::make_unique<ZqwlCanfdDriver>();
  }
  if (type == "zlg_canfd") {
    return std::make_unique<CanfdDriver>();
  }
  return nullptr;
}

}  // namespace ghand_comm
