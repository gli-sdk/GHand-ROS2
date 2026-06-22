#include "communication/communication.hpp"

#include <array>

#include "communication/canfd_driver.hpp"
#include "communication/ethercat_driver.hpp"
#include "communication/rs485_driver.hpp"
#include "communication/zqwl_canfd_driver.hpp"

namespace ghand_comm
{

namespace
{

constexpr size_t kJointCount = static_cast<size_t>(JointId::NUM_JOINTS);

const std::array<std::string, kJointCount + 1> kJointIdNames = {
  "thumb_dip",
  "thumb_mcp",
  "thumb_tmc_fe",
  "thumb_tmc_aa",
  "thumb_tmc_ps",
  "index_dip",
  "index_pip",
  "index_mcp",
  "index_mcp_aa",
  "middle_dip",
  "middle_pip",
  "middle_mcp",
  "ring_dip",
  "ring_pip",
  "ring_mcp",
  "little_dip",
  "little_pip",
  "little_mcp",
  "UNKNOWN",
};

const std::map<JointId, uint16_t> kHoldingRegMap = {
  {JointId::THUMB_MCP, 0x0011},
  {JointId::THUMB_TMC_FE, 0x0013},
  {JointId::THUMB_TMC_AA, 0x0015},
  {JointId::THUMB_TMC_PS, 0x0017},
  {JointId::INDEX_PIP, 0x0019},
  {JointId::INDEX_MCP, 0x001B},
  {JointId::INDEX_MCP_AA, 0x001D},
  {JointId::MIDDLE_PIP, 0x001F},
  {JointId::MIDDLE_MCP, 0x0021},
  {JointId::RING_PIP, 0x0023},
  {JointId::RING_MCP, 0x0025},
  {JointId::LITTLE_PIP, 0x0027},
  {JointId::LITTLE_MCP, 0x0029},
  
};

}  // namespace

std::optional<JointId> joint_name_to_id(const std::string & name)
{
  if (name == "thumb_tmc_ps") return JointId::THUMB_TMC_PS;
  if (name == "thumb_tmc_aa") return JointId::THUMB_TMC_AA;
  if (name == "thumb_tmc_fe") return JointId::THUMB_TMC_FE;
  if (name == "thumb_mcp") return JointId::THUMB_MCP;
  if (name == "thumb_dip") return JointId::THUMB_DIP;

  if (name == "index_mcp_aa") return JointId::INDEX_MCP_AA;
  if (name == "index_mcp") return JointId::INDEX_MCP;
  if (name == "index_pip") return JointId::INDEX_PIP;
  if (name == "index_dip") return JointId::INDEX_DIP;

  if (name == "middle_mcp") return JointId::MIDDLE_MCP;
  if (name == "middle_pip") return JointId::MIDDLE_PIP;
  if (name == "middle_dip") return JointId::MIDDLE_DIP;

  if (name == "ring_mcp") return JointId::RING_MCP;
  if (name == "ring_pip") return JointId::RING_PIP;
  if (name == "ring_dip") return JointId::RING_DIP;

  if (name == "little_mcp") return JointId::LITTLE_MCP;
  if (name == "little_pip") return JointId::LITTLE_PIP;
  if (name == "little_dip") return JointId::LITTLE_DIP;

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
  if (type == "zqwl_canfd")
  {
    return std::make_unique<ZqwlCanfdDriver>();
  }
  return nullptr;
}

}  // namespace ghand_comm
