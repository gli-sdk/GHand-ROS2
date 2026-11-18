// Copyright (c) 2026 GHand ROS2 Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "ghand_hardware_interface/communication/communication.hpp"

#include <array>
#include <map>
#include <memory>
#include <string>

#include "ghand_hardware_interface/sdk_session.hpp"

namespace ghand_comm {

namespace {

constexpr size_t kJointCount = static_cast<size_t>(JointId::NUM_JOINTS);

const std::array<std::string, kJointCount + 1> kJointIdNames = {
    "Thumb_IP",  "Thumb_MCP",  "Thumb_TMC_FE", "Thumb_TMC_AA", "Thumb_TMC_PS",
    "FF_DIP",  "FF_PIP",  "FF_MCP",    "FF_MCP_AA", "MF_DIP",
    "MF_PIP", "MF_MCP", "RF_DIP",     "RF_PIP",     "RF_MCP",
    "LF_DIP", "LF_PIP", "LF_MCP",   "UNKNOWN",
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

  if (canonical_name == "THUMB_MCP") return JointId::THUMB_MCP;
  if (canonical_name == "THUMB_TMC_FE") return JointId::THUMB_TMC_FE;
  if (canonical_name == "THUMB_TMC_AA") return JointId::THUMB_TMC_AA;
  if (canonical_name == "FF_IP") return JointId::FF_PIP;
  if (canonical_name == "MF_IP") return JointId::MF_PIP;
  if (canonical_name == "RF_IP") return JointId::RF_PIP;
  if (canonical_name == "LF_IP") return JointId::LF_PIP;

  if (canonical_name == "Thumb_TMC_PS") return JointId::THUMB_TMC_PS;
  if (canonical_name == "Thumb_TMC_AA") return JointId::THUMB_TMC_AA;
  if (canonical_name == "Thumb_TMC_FE") return JointId::THUMB_TMC_FE;
  if (canonical_name == "Thumb_MCP") return JointId::THUMB_MCP;
  if (canonical_name == "Thumb_IP") return JointId::THUMB_IP;

  if (canonical_name == "FF_MCP_AA") return JointId::FF_MCP_AA;
  if (canonical_name == "FF_MCP") return JointId::FF_MCP;
  if (canonical_name == "FF_PIP") return JointId::FF_PIP;
  if (canonical_name == "FF_DIP") return JointId::FF_DIP;

  if (canonical_name == "MF_MCP") return JointId::MF_MCP;
  if (canonical_name == "MF_PIP") return JointId::MF_PIP;
  if (canonical_name == "MF_DIP") return JointId::MF_DIP;

  if (canonical_name == "RF_MCP") return JointId::RF_MCP;
  if (canonical_name == "RF_PIP") return JointId::RF_PIP;
  if (canonical_name == "RF_DIP") return JointId::RF_DIP;

  if (canonical_name == "LF_MCP") return JointId::LF_MCP;
  if (canonical_name == "LF_PIP") return JointId::LF_PIP;
  if (canonical_name == "LF_DIP") return JointId::LF_DIP;

  return std::nullopt;
}

const std::string& joint_id_to_name(JointId id) {
  const size_t index = static_cast<size_t>(id);
  if (index < kJointIdNames.size()) {
    return kJointIdNames[index];
  }
  return kJointIdNames.back();
}

std::unique_ptr<CommunicationBase> create_communication(const std::string& type) {
  if (type == "ethercat" || type == "rs485" || type == "canfd" || type == "zqwl_canfd")
    return std::make_unique<SdkSession>(type == "zqwl_canfd" ? "canfd" : type);
  return nullptr;
}
}  // namespace ghand_comm
