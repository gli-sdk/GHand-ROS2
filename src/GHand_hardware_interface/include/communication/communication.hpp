#ifndef GHAND_HARDWARE__COMMUNICATION_HPP_
#define GHAND_HARDWARE__COMMUNICATION_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace ghand_comm
{

enum class JointId : uint8_t
{
  THUMB_DIP = 0,
  THUMB_PIP,
  THUMB_MCP,
  THUMB_SWING,
  THUMB_ROTATION,
  FF_DIP,
  FF_PIP,
  FF_MCP,
  FF_SWING,
  MF_DIP,
  MF_PIP,
  MF_MCP,
  RF_DIP,
  RF_PIP,
  RF_MCP,
  LF_DIP,
  LF_PIP,
  LF_MCP,
  NUM_JOINTS
};

struct Force
{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct JointState
{
  float position = 0.0f;  // degrees
  float velocity = 0.0f;  // percent (-100~100)
  uint8_t status = 0;
  uint8_t error = 0;
};

struct JointCommand
{
  JointId id = JointId::NUM_JOINTS;
  float position = 0.0f;  // degrees
  int8_t velocity = 0;    // percent (-100~100)
  int8_t torque = 0;      // percent (-100~100)
};

struct TactileRegion
{
  std::string name;
  Force resultant_force;
  std::vector<Force> distributed_forces;
};

struct CommunicationData
{
  std::vector<JointState> joints;
  std::vector<TactileRegion> tactile_regions;
  bool valid = false;
};

class CommunicationBase
{
public:
  virtual ~CommunicationBase() = default;

  virtual bool init(const std::unordered_map<std::string, std::string> & params) = 0;
  virtual bool read(CommunicationData & out_data) = 0;
  virtual bool write(const std::vector<JointCommand> & commands) = 0;
  virtual void close() = 0;
  virtual const char * get_name() const = 0;
};

std::optional<JointId> joint_name_to_id(const std::string & name);
const std::string & joint_id_to_name(JointId id);
const std::map<JointId, uint16_t> & holding_register_map();
std::unique_ptr<CommunicationBase> create_communication(const std::string & type);

}  // namespace ghand_comm

#endif  // GHAND_HARDWARE__COMMUNICATION_HPP_
