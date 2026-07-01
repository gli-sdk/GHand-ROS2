// Copyright (c) 2026 GHand ROS2 Maintainers
// SPDX-License-Identifier: MIT

#ifndef GHAND_HARDWARE_INTERFACE_COMMUNICATION_COMMUNICATION_HPP_
#define GHAND_HARDWARE_INTERFACE_COMMUNICATION_COMMUNICATION_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace ghand_comm {

enum class JointId : uint8_t {
  THUMB_DIP = 0,
  THUMB_MCP,
  THUMB_TMC_FE,
  THUMB_TMC_AA,
  THUMB_TMC_PS,
  INDEX_DIP,
  INDEX_PIP,
  INDEX_MCP,
  INDEX_MCP_AA,
  MIDDLE_DIP,
  MIDDLE_PIP,
  MIDDLE_MCP,
  RING_DIP,
  RING_PIP,
  RING_MCP,
  LITTLE_DIP,
  LITTLE_PIP,
  LITTLE_MCP,
  NUM_JOINTS
};

struct Force {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct JointState {
  float position = 0.0f;  // degrees
  float velocity = 0.0f;  // percent (-100~100)
  uint8_t status = 0;
  uint8_t error = 0;
};

struct JointCommand {
  JointId id = JointId::NUM_JOINTS;
  float position = 0.0f;  // degrees
  int8_t velocity = 0;    // percent (-100~100)
  int8_t torque = 0;      // percent (-100~100)
};

struct TactileRegion {
  std::string name;
  Force resultant_force;
  std::vector<Force> distributed_forces;
};

enum class CtrlMode : uint8_t {
  POSITION = 0,
  TORQUE = 1,
  SPEED = 2,
};

struct HandState {
  uint8_t state = 0;
  uint8_t error = 0;
  int16_t temperature = 0;
};

struct CommunicationData {
  HandState hand_state;
  std::vector<JointState> joints;
  std::vector<TactileRegion> tactile_regions;
  bool valid = false;
};

class CommunicationBase {
 public:
  virtual ~CommunicationBase() = default;

  virtual bool init(
      const std::unordered_map<std::string, std::string>& params) = 0;
  virtual bool read(CommunicationData& out_data) = 0;
  virtual bool write(const std::vector<JointCommand>& commands) = 0;
  virtual void close() = 0;
  virtual const char* get_name() const = 0;
};

std::optional<JointId> joint_name_to_id(const std::string& name);
const std::string& joint_id_to_name(JointId id);
const std::map<JointId, uint16_t>& holding_register_map();
std::unique_ptr<CommunicationBase> create_communication(
    const std::string& type);

}  // namespace ghand_comm

#endif  // GHAND_HARDWARE_INTERFACE_COMMUNICATION_COMMUNICATION_HPP_
