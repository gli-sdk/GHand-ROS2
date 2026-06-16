#include "communication/communication.hpp"

#include <array>

#include "communication/canfd_driver.hpp"
#include "communication/ethercat_driver.hpp"
#include "communication/rs485_driver.hpp"

namespace ghand_comm
{

namespace
{

constexpr size_t kJointCount = static_cast<size_t>(JointId::NUM_JOINTS);

const std::array<std::string, kJointCount + 1> kJointIdNames = {
  "THUMB_DIP",
  "THUMB_PIP",
  "THUMB_MCP",
  "THUMB_SWING",
  "THUMB_ROTATION",
  "FF_DIP",
  "FF_PIP",
  "FF_MCP",
  "FF_SWING",
  "MF_DIP",
  "MF_PIP",
  "MF_MCP",
  "RF_DIP",
  "RF_PIP",
  "RF_MCP",
  "LF_DIP",
  "LF_PIP",
  "LF_MCP",
  "UNKNOWN",
};

const std::map<JointId, uint16_t> kHoldingRegMap = {
  {JointId::THUMB_PIP, 0x0011},
  {JointId::THUMB_MCP, 0x0013},
  {JointId::THUMB_SWING, 0x0015},
  {JointId::THUMB_ROTATION, 0x0017},
  {JointId::FF_DIP, 0x0019},
  {JointId::FF_PIP, 0x001B},
  {JointId::FF_MCP, 0x001D},
  {JointId::FF_SWING, 0x001F},
  {JointId::MF_DIP, 0x0021},
  {JointId::MF_PIP, 0x0023},
  {JointId::MF_MCP, 0x0025},
  {JointId::RF_DIP, 0x0027},
  {JointId::RF_PIP, 0x0029},
  {JointId::RF_MCP, 0x002B},
  {JointId::LF_DIP, 0x002D},
  {JointId::LF_PIP, 0x002F},
  {JointId::LF_MCP, 0x0031},
};

}  // namespace

std::optional<JointId> joint_name_to_id(const std::string & name)
{
  if (name == "thumb_tmc_ps") return JointId::THUMB_ROTATION;
  if (name == "thumb_tmc_aa") return JointId::THUMB_SWING;
  if (name == "thumb_tmc_fe") return JointId::THUMB_MCP;
  if (name == "thumb_mcp") return JointId::THUMB_PIP;
  if (name == "thumb_dip") return JointId::THUMB_DIP;

  if (name == "index_mcp_aa") return JointId::FF_SWING;
  if (name == "index_mcp") return JointId::FF_MCP;
  if (name == "index_pip") return JointId::FF_PIP;
  if (name == "index_dip") return JointId::FF_DIP;

  if (name == "middle_mcp") return JointId::MF_MCP;
  if (name == "middle_pip") return JointId::MF_PIP;
  if (name == "middle_dip") return JointId::MF_DIP;

  if (name == "ring_mcp") return JointId::RF_MCP;
  if (name == "ring_pip") return JointId::RF_PIP;
  if (name == "ring_dip") return JointId::RF_DIP;

  if (name == "little_mcp") return JointId::LF_MCP;
  if (name == "little_pip") return JointId::LF_PIP;
  if (name == "little_dip") return JointId::LF_DIP;

  return std::nullopt;
}

const std::string & joint_id_to_name(JointId id)
{
  const size_t index = static_cast<size_t>(id);
  if (index < kJointIdNames.size())
  {
    return kJointIdNames[index];
  }
  return kJointIdNames.back();
}

const std::map<JointId, uint16_t> & holding_register_map()
{
  return kHoldingRegMap;
}

std::unique_ptr<CommunicationBase> create_communication(const std::string & type)
{
  if (type == "ethercat")
  {
    return std::make_unique<EthercatDriver>();
  }
  if (type == "rs485")
  {
    return std::make_unique<Rs485Driver>();
  }
  if (type == "canfd")
  {
    return std::make_unique<CanfdDriver>();
  }
  return nullptr;
}

}  // namespace ghand_comm
