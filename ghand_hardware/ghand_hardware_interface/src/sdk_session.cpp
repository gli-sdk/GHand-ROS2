// SPDX-License-Identifier: Apache-2.0
#include "ghand_hardware_interface/sdk_session.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "ghand_hardware_interface/communication/product_config.hpp"
#include "ghand_hardware_interface/ros2_control_adapter.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ghand_comm {
namespace {
using Params = std::unordered_map<std::string, std::string>;
std::string parameter(const Params& p, const std::string& key, const std::string& fallback) {
  const auto it = p.find(key);
  return it == p.end() || it->second.empty() ? fallback : it->second;
}
int integer(const Params& p, const std::string& key, int fallback) {
  const auto value = parameter(p, key, std::to_string(fallback));
  size_t end = 0;
  const int result = std::stoi(value, &end, 0);
  if (end != value.size()) throw std::invalid_argument("Invalid " + key);
  return result;
}
ghand::RS485BaudRate rs485_rate(int rate) {
  switch (rate) {
    case 57600: return ghand::RS485BaudRate::RATE_57600;
    case 115200: return ghand::RS485BaudRate::RATE_115200;
    case 230400: return ghand::RS485BaudRate::RATE_230400;
    case 460800: return ghand::RS485BaudRate::RATE_460800;
    case 921600: return ghand::RS485BaudRate::RATE_921600;
    case 1000000: return ghand::RS485BaudRate::RATE_1000000;
    default: throw std::invalid_argument("Unsupported SDK RS485 baud_rate");
  }
}
ghand::CANFDBitTiming can_timing(int arbitration, int data) {
  const std::pair<int, int> rates[] = {{500000,800000}, {500000,1000000},
    {500000,2000000}, {500000,4000000}, {1000000,1000000}, {1000000,2000000},
    {1000000,4000000}, {1000000,5000000}};
  for (size_t i = 0; i < 8; ++i)
    if (rates[i] == std::make_pair(arbitration, data))
      return static_cast<ghand::CANFDBitTiming>(i);
  throw std::invalid_argument("Unsupported SDK CANFD baud_rate/data_rate pair");
}
bool checked(const ghand::Result& result, const char* operation) {
  if (!result) RCLCPP_ERROR(rclcpp::get_logger("GHandSdk"), "%s failed: SDK code=%d %s",
      operation, static_cast<int>(result.code), result.message.c_str());
  return static_cast<bool>(result);
}
}  // namespace

bool SdkSession::init(const Params& params) {
  close();
  try {
    const auto product = parameter(params, "product_name", "ghand5");
    const auto* metadata = find_product_config(product);
    if (!metadata) throw std::invalid_argument("Unknown product_name");
    const auto mode = parse_hand_control_mode(parameter(params, "control_mode", "position"));
    if (!mode) throw std::invalid_argument("Invalid control_mode");
    mode_ = *mode;
    const auto type = transport_ == "ethercat" ? ghand::CommType::ETHERCAT
      : transport_ == "rs485" ? ghand::CommType::RS485 : ghand::CommType::CANFD;
    const int node = integer(params, "node_id", 0);
    if (node < 0 || node > 255) throw std::invalid_argument("node_id must be 0..255");
    const auto device = parameter(params, transport_ == "ethercat" ? "ifname" : "device", "auto");
    master_ = ghand::Master::Create(metadata->lite1() ? ghand::ProductType::GHandLite1
                                                  : ghand::ProductType::GHand5, type);
    if (!master_) throw std::runtime_error("SDK Master::Create failed");
    ghand::ResultOf<std::vector<ghand::GHand*>> connection;
    if (type == ghand::CommType::RS485) {
      if (integer(params, "data_bits", 8) != 8 || integer(params, "stop_bits", 1) != 1 ||
          parameter(params, "parity", "N") != "N")
        throw std::invalid_argument("SDK RS485 requires 8N1");
      connection = master_->Connect(device, static_cast<uint8_t>(node),
                                    rs485_rate(integer(params, "baud_rate", 1000000)));
    } else if (type == ghand::CommType::CANFD) {
      connection = master_->Connect(device, static_cast<uint8_t>(node),
          can_timing(integer(params, "baud_rate", 1000000), integer(params, "data_rate", 5000000)));
    } else {
      connection = master_->Connect(device);
    }
    if (!checked(connection, "Connect")) { close(); return false; }
    const auto side = parameter(params, "hand_side", "");
    if (side == "left" || side == "right") {
      hand_ = master_->hand(side == "left" ? ghand::HandType::LEFT_HAND : ghand::HandType::RIGHT_HAND);
    } else if (params.count("slave_index")) {
      const int index = integer(params, "slave_index", 1);
      if (index < 1 || index > 65535) throw std::invalid_argument("Invalid slave_index");
      hand_ = master_->hand(static_cast<uint16_t>(index));
    } else if (connection.value.size() == 1) {
      hand_ = connection.value.front();
    }
    if (!hand_) throw std::runtime_error("Select hand_side or slave_index when the SDK discovers multiple hands");
    feedback_ = std::make_shared<Feedback>();
    const std::weak_ptr<Feedback> weak = feedback_;
    const auto expected = metadata->joints;
    hand_->SetJointsCallback([weak, expected](const std::vector<JointData>& joints) {
      const auto cache = weak.lock();
      if (!cache) return;
      // A partial or duplicate snapshot must not refresh the whole hand's age.
      if (joints.size() != expected.size()) return;
      for (const auto& joint : joints) if (!std::isfinite(joint.angle)) return;
      for (const auto id : expected)
        if (std::count_if(joints.begin(), joints.end(), [id](const auto& j) { return j.id == id; }) != 1)
          return;
      std::lock_guard<std::mutex> lock(cache->mutex);
      cache->data.device.joints = joints;
      cache->data.received = std::chrono::steady_clock::now();
      cache->data.valid = true;
    });
    hand_->SetHandDataCallback([weak](const HandData& data) {
      if (const auto cache = weak.lock()) {
        std::lock_guard<std::mutex> lock(cache->mutex);
        cache->data.device.hand = data;
        cache->hand_received = std::chrono::steady_clock::now();
      }
    });
    hand_->SetTactileDataCallback([weak](const std::map<TactileSensorId, TactileData>& data) {
      if (const auto cache = weak.lock()) {
        std::lock_guard<std::mutex> lock(cache->mutex);
        cache->data.device.tactile = data;
        cache->tactile_received = std::chrono::steady_clock::now();
      }
    });
    return true;
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("GHandSdk"), "SDK configuration failed: %s", error.what());
    close();
    return false;
  }
}

bool SdkSession::read(CommunicationData& data) {
  if (!master_ || !master_->IsConnected() || !feedback_) return false;
  std::lock_guard<std::mutex> lock(feedback_->mutex);
  data = feedback_->data;
  const auto now = std::chrono::steady_clock::now();
  data.hand_state_valid = now - feedback_->hand_received < std::chrono::milliseconds(500);
  if (now - feedback_->tactile_received >= std::chrono::milliseconds(500)) data.device.tactile.clear();
  return data.valid;
}
bool SdkSession::write(const std::vector<JointCommand>& commands) {
  if (!master_ || !master_->IsConnected() || !hand_) return false;
  return commands.empty() || checked(hand_->MoveJoints(commands, mode_), "MoveJoints");
}
bool SdkSession::stop() {
  return !hand_ || checked(hand_->Stop(), "Stop");
}
void SdkSession::close() {
  // Callbacks capture a weak cache, never this session or the hardware plugin.
  feedback_.reset();
  if (hand_) {
    hand_->SetJointsCallback({});
    hand_->SetHandDataCallback({});
    hand_->SetTactileDataCallback({});
  }
  hand_ = nullptr;
  if (master_) { checked(master_->Disconnect(), "Disconnect"); master_.reset(); }
}
}  // namespace ghand_comm
