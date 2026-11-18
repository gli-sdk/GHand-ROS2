// SPDX-License-Identifier: Apache-2.0
#ifndef GHAND_HARDWARE_INTERFACE_COMMUNICATION_COMMUNICATION_HPP_
#define GHAND_HARDWARE_INTERFACE_COMMUNICATION_COMMUNICATION_HPP_
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include "ghand_hardware_interface/communication/sdk_types.hpp"
namespace ghand_comm {
struct CommunicationData {
  DeviceData device;
  std::chrono::steady_clock::time_point received{};
  bool hand_state_valid = false;
  bool joint_control_info_available = true;
  bool valid = false;
};
// ROS-side session boundary. All device operations are implemented by GHand SDK.
class CommunicationBase {
 public:
  virtual ~CommunicationBase() = default;
  virtual bool init(const std::unordered_map<std::string, std::string>& params) = 0;
  virtual bool read(CommunicationData& data) = 0;
  virtual bool write(const std::vector<JointCommand>& commands) = 0;
  virtual bool stop() = 0;
  virtual void close() = 0;
  virtual const char* get_name() const = 0;
};
std::optional<JointId> joint_name_to_id(const std::string& name);
const std::string& joint_id_to_name(JointId id);
std::unique_ptr<CommunicationBase> create_communication(const std::string& type);
}  // namespace ghand_comm
#endif
