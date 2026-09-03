// Copyright (c) 2026 GHand ROS2 Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "ghand_hardware_interface/communication/zqwl_canfd_driver.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ghand_hardware_interface/communication/canfd_protocol.hpp"
#include "ghand_hardware_interface/communication/communication.hpp"
#include "ghand_hardware_interface/communication/modbus_codec.hpp"
#include "ghand_hardware_interface/communication/util.hpp"
#include "rclcpp/logging.hpp"

namespace ghand_comm {

namespace {

// parse_int_default() is provided by
// ghand_hardware_interface/communication/util.hpp.

speed_t map_baud_rate(uint32_t baud) {
  switch (baud) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 500000:
      return B500000;
    case 576000:
      return B576000;
    case 921600:
      return B921600;
    case 1000000:
      return B1000000;
    case 1152000:
      return B1152000;
    case 1500000:
      return B1500000;
    case 2000000:
      return B2000000;
    case 2500000:
      return B2500000;
    case 3000000:
      return B3000000;
    case 3500000:
      return B3500000;
    case 4000000:
      return B4000000;
    default:
      return B2000000;
  }
}

const std::map<uint32_t, uint8_t>& common_bitrate_codes() {
  static const std::map<uint32_t, uint8_t> codes = {
      {1000000, 0x0}, {800000, 0x1}, {500000, 0x2}, {400000, 0x3},
      {250000, 0x4},  {200000, 0x5}, {125000, 0x6}, {100000, 0x7},
      {50000, 0x8},   {40000, 0x9},  {25000, 0xA},  {20000, 0xB},
      {15000, 0xC},   {10000, 0xD},  {5000, 0xE}};
  return codes;
}

const std::map<uint32_t, uint8_t>& common_data_bitrate_codes() {
  static const std::map<uint32_t, uint8_t> codes = {
      {5000000, 0x0}, {4000000, 0x1}, {2000000, 0x2}, {1000000, 0x3},
      {800000, 0x4},  {500000, 0x5},  {400000, 0x6},  {250000, 0x7},
      {200000, 0x8},  {125000, 0x9},  {100000, 0xA}};
  return codes;
}

int payload_length_from_adapter_dlc(uint8_t value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 12:
    case 16:
    case 20:
    case 24:
    case 32:
    case 48:
    case 64:
      return value;
    default:
      return -1;
  }
}

}  // namespace

// GHand-Cpp-SDK connection-management payloads.
constexpr std::array<uint8_t, 6> kEstablishConnectionPayload = {
    0x00, 0x01, 0x00, 0x02, 0x00, 0x00};
constexpr std::array<uint8_t, 6> kDeleteConnectionPayload = {
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
constexpr uint8_t kDefaultSrcId = 0x0A;
constexpr uint8_t kDefaultDstId = 0x31;

struct ZqwlCanfdDriver::Impl {
  std::string device_path_ = "/dev/ttyACM0";
  uint32_t serial_baud_rate_ = 2000000;
  uint32_t can_bitrate_ = 1000000;
  uint32_t dbit_rate_ = 5000000;
  uint32_t response_timeout_ms_ = 500;
  uint8_t can_channel_ = 0;
  uint8_t preferred_node_id_ = 0x31;
  uint8_t src_id_ = kDefaultSrcId;
  uint8_t dst_id_ = kDefaultDstId;
  CtrlMode control_mode_ = CtrlMode::POSITION;
  bool control_mode_written_ = false;

  // Timeout tracking: if no CAN data received within this window,
  // write() returns false to trigger safe deactivation.
  std::chrono::steady_clock::time_point last_rx_time_;
  bool rx_timeout_initialized_ = false;
  static constexpr auto kRxTimeout = std::chrono::seconds(10);

  int serial_fd_ = -1;
  std::atomic<bool> connected_{false};
  std::atomic<bool> handshake_done_{false};
  std::atomic<bool> run_thread_{false};
  std::thread recv_thread_;

  std::mutex data_mutex_;
  std::mutex io_mutex_;
  CommunicationData latest_data_;

  std::vector<JointId> valid_joints_;
  std::vector<std::pair<std::string, size_t>> tactile_regions_;
  std::map<JointId, JointCommand> last_sent_commands_;
  std::chrono::steady_clock::time_point last_command_time_;
  static constexpr auto kMinCommandInterval = std::chrono::milliseconds(40);

  // Persistent receive buffer shared by handshake and polling reads.
  std::vector<uint8_t> rx_buffer_;
  size_t rx_parse_offset_ = 0;

  bool OpenSerial();
  void CloseSerial();
  bool SendConfigFrame(uint8_t func_code, const uint8_t* data);
  bool SetCanParameters();
  bool StartCanChannel();
  bool TransmitFrame(uint32_t can_id, const uint8_t* data, uint8_t len);
  bool WriteRegisters(uint16_t addr, const std::vector<uint8_t>& data,
                      uint32_t timeout_ms = 0);
  bool WriteHoldingRegister(uint16_t addr, uint16_t value);

  bool EstablishConnection();
  void DeleteConnection();
  bool ReadInputRegisters(uint16_t addr, uint16_t count,
                          std::vector<uint8_t>& out);
  bool ReadModbusResponse(uint8_t expected_func, std::vector<uint8_t>& out,
                          uint32_t timeout_ms,
                          uint8_t* exception_code = nullptr);
  bool ReadNextCanFrame(canfd::Frame* out,
                        std::chrono::steady_clock::time_point deadline);

  void ReceiveLoop();
  void PollHandState();
  void PollJoints();
  void PollTactile();
};

ZqwlCanfdDriver::ZqwlCanfdDriver() : impl_(std::make_unique<Impl>()) {}

ZqwlCanfdDriver::~ZqwlCanfdDriver() { close(); }

const char* ZqwlCanfdDriver::get_name() const { return "canfd"; }

bool ZqwlCanfdDriver::init(
    const std::unordered_map<std::string, std::string>& params) {
  auto it = params.find("device");
  if (it != params.end() && !it->second.empty()) {
    impl_->device_path_ = it->second;
  }

  // GHand-Cpp-SDK accepts "/dev/ttyACM0:0".  Keep can_channel as an
  // explicit ROS parameter too, but let the device suffix select it when
  // present.
  const size_t channel_separator = impl_->device_path_.rfind(':');
  if (channel_separator != std::string::npos) {
    const std::string channel = impl_->device_path_.substr(channel_separator + 1);
    if (!channel.empty()) {
      impl_->can_channel_ = static_cast<uint8_t>(parse_int_default(channel, 0));
      impl_->device_path_.resize(channel_separator);
    }
  }

  impl_->serial_baud_rate_ = static_cast<uint32_t>(parse_int_default(
      params.count("serial_baud_rate") ? params.at("serial_baud_rate") : "",
      2000000));
  impl_->can_bitrate_ = static_cast<uint32_t>(parse_int_default(
      params.count("bitrate")   ? params.at("bitrate")
      : params.count("baud_rate") ? params.at("baud_rate")
                                    : "",
      1000000));
  impl_->dbit_rate_ = static_cast<uint32_t>(
      parse_int_default(params.count("data_rate")   ? params.at("data_rate")
                        : params.count("dbit_rate") ? params.at("dbit_rate")
                                                    : "",
                        5000000));
  impl_->response_timeout_ms_ = static_cast<uint32_t>(parse_int_default(
      params.count("response_timeout_ms") ? params.at("response_timeout_ms")
                                            : "",
      500));
  if (params.count("can_channel")) {
    impl_->can_channel_ = static_cast<uint8_t>(
        parse_int_default(params.at("can_channel"), impl_->can_channel_));
  }
  impl_->preferred_node_id_ = static_cast<uint8_t>(
      parse_int_default(params.count("node_id")     ? params.at("node_id")
                        : params.count("device_id") ? params.at("device_id")
                                                    : "",
                        49));

  auto cm = params.find("control_mode");
  if (cm != params.end()) {
    if (cm->second == "current" || cm->second == "CURRENT" || cm->second == "1") {
      impl_->control_mode_ = CtrlMode::CURRENT;
    } else if (cm->second == "speed" || cm->second == "2") {
      impl_->control_mode_ = CtrlMode::SPEED;
    } else {
      impl_->control_mode_ = CtrlMode::POSITION;
    }
  }

  auto jt = params.find("joint_names");
  if (jt != params.end()) {
    for (const auto& name : split(jt->second, ',')) {
      const auto id = joint_name_to_id(name);
      if (id.has_value()) {
        impl_->valid_joints_.push_back(id.value());
      }
    }
  }
  if (impl_->valid_joints_.empty()) {
    for (uint8_t i = 0; i < static_cast<uint8_t>(JointId::NUM_JOINTS); ++i) {
      impl_->valid_joints_.push_back(static_cast<JointId>(i));
    }
  }

  auto tt = params.find("tactile_region_counts");
  if (tt != params.end()) {
    impl_->tactile_regions_ = parse_tactile_regions(tt->second);
  }

  // Open the configured adapter only.  Falling back to another tty can bind
  // the left ROS system to the right-hand adapter in a dual-hand setup.
  if (!impl_->OpenSerial()) {
    return false;
  }

  if (!impl_->SetCanParameters()) {
    impl_->CloseSerial();
    return false;
  }

  if (!impl_->StartCanChannel()) {
    impl_->CloseSerial();
    return false;
  }

  // Match GHand-Cpp-SDK Connect(): clear stale sessions for both supported
  // node IDs before establishing a fresh one.
  const uint8_t original_dst = impl_->dst_id_;
  for (uint8_t dst : {static_cast<uint8_t>(0x31),
                      static_cast<uint8_t>(0x32)}) {
    impl_->dst_id_ = dst;
    impl_->DeleteConnection();
  }
  impl_->dst_id_ = original_dst;

  if (!impl_->EstablishConnection()) {
    impl_->CloseSerial();
    return false;
  }

  impl_->connected_.store(true);
  impl_->handshake_done_.store(true);
  impl_->last_rx_time_ = std::chrono::steady_clock::now();
  impl_->rx_timeout_initialized_ = true;

  RCLCPP_INFO(
      rclcpp::get_logger("zqwl_canfd"),
      "initialized from GHand-Cpp-SDK: device=%s serial_baud_rate=%u "
      "bitrate=%u data_rate=%u node_id=0x%02X channel=%u",
      impl_->device_path_.c_str(), impl_->serial_baud_rate_,
      impl_->can_bitrate_, impl_->dbit_rate_, impl_->dst_id_,
      impl_->can_channel_);

  // Seed latest_data_ so read() returns valid zero-values immediately,
  // avoiding the Jazzy framework segfault when controllers load without CAN
  // data.
  {
    CommunicationData seed;
    seed.valid = true;
    seed.joints.resize(impl_->valid_joints_.size());
    for (auto& js : seed.joints) {
      js.position = 0.0f;
      js.velocity = 0.0f;
      js.status = 0;
      js.error = 0;
    }
    impl_->latest_data_ = std::move(seed);
  }

  // Discard any stale bytes that arrived before the polling loop starts.
  tcflush(impl_->serial_fd_, TCIOFLUSH);
  impl_->rx_buffer_.clear();
  impl_->rx_parse_offset_ = 0;

  impl_->run_thread_.store(true);
  impl_->recv_thread_ = std::thread([this]() { impl_->ReceiveLoop(); });

  return true;
}

bool ZqwlCanfdDriver::read(CommunicationData& out_data) {
  std::lock_guard<std::mutex> lock(impl_->data_mutex_);
  out_data = impl_->latest_data_;
  bool ret = impl_->connected_.load() && out_data.valid;
  return ret;
}

bool ZqwlCanfdDriver::write(const std::vector<JointCommand>& commands) {
  // Safety: if no CAN data has been received within the timeout window,
  // return false so the framework can deactivate cleanly instead of
  // triggering an inexplicable segfault in the controller_manager RT loop.
  {
    std::lock_guard<std::mutex> data_lock(impl_->data_mutex_);
    if (impl_->rx_timeout_initialized_) {
      const auto elapsed =
          std::chrono::steady_clock::now() - impl_->last_rx_time_;
      if (elapsed > Impl::kRxTimeout) {
        impl_->connected_.store(false);
        return false;
      }
    }
  }

  if (!impl_->connected_.load() || commands.empty()) {
    return true;
  }

  bool commands_changed = commands.size() != impl_->last_sent_commands_.size();
  for (const auto& cmd : commands) {
    const auto previous = impl_->last_sent_commands_.find(cmd.id);
    if (previous == impl_->last_sent_commands_.end() ||
        previous->second.position != cmd.position ||
        previous->second.velocity != cmd.velocity ||
        previous->second.current != cmd.current ||
        previous->second.kp != cmd.kp ||
        previous->second.kd != cmd.kd ||
        previous->second.mode != cmd.mode ||
        previous->second.stop != cmd.stop) {
      commands_changed = true;
      break;
    }
  }
  if (!commands_changed) {
    return true;
  }

  const auto now = std::chrono::steady_clock::now();
  if (impl_->last_command_time_.time_since_epoch().count() != 0 &&
      now - impl_->last_command_time_ < Impl::kMinCommandInterval) {
    return true;
  }

  // The control mode is constant for the lifetime of this backend. Sending it
  // once avoids an extra request/response transaction in every ros2_control
  // cycle.
  if (!impl_->control_mode_written_) {
    const uint16_t mode_value =
        static_cast<uint16_t>(static_cast<uint8_t>(impl_->control_mode_) << 8);
    if (!impl_->WriteHoldingRegister(0x0260, mode_value)) {
      return false;
    }
    impl_->control_mode_written_ = true;
  }

  // Active-joint command blocks start at 0x0261 with 6 registers per joint.
  // ros2_control supplies all commandable joints on every update, so combine
  // adjacent joint commands into Modbus blocks. This preserves the SDK wire
  // encoding while reducing the normal G5 update from 12 CANFD transactions
  // to one and leaves time for the background feedback polling thread.
  std::map<uint16_t, std::vector<uint8_t>> encoded_by_address;
  const auto& reg_map = holding_register_map();
  for (const auto& cmd : commands) {
    auto it = reg_map.find(cmd.id);
    if (it == reg_map.end()) continue;

    const auto regs = encode_joint_command(cmd, impl_->control_mode_);
    std::vector<uint8_t> data = registers_to_bytes(regs);

    encoded_by_address[it->second] = data;
  }

  uint16_t block_start = 0;
  uint16_t next_address = 0;
  std::vector<uint8_t> block_data;
  auto flush_block = [&]() {
    if (block_data.empty()) return true;
    const bool ok = impl_->WriteRegisters(block_start, block_data);
    block_data.clear();
    return ok;
  };

  for (const auto& entry : encoded_by_address) {
    if (block_data.empty()) {
      block_start = entry.first;
      next_address = entry.first;
    } else if (entry.first != next_address) {
      if (!flush_block()) return false;
      block_start = entry.first;
      next_address = entry.first;
    }
    block_data.insert(block_data.end(), entry.second.begin(), entry.second.end());
    next_address = static_cast<uint16_t>(entry.first + entry.second.size() / 2);
  }
  if (!flush_block()) {
    return false;
  }
  impl_->last_sent_commands_.clear();
  for (const auto& cmd : commands) {
    impl_->last_sent_commands_[cmd.id] = cmd;
  }
  impl_->last_command_time_ = now;
  return true;
}

void ZqwlCanfdDriver::close() {
  impl_->run_thread_ = false;
  if (impl_->recv_thread_.joinable()) {
    impl_->recv_thread_.join();
  }
  if (impl_->handshake_done_.load()) {
    impl_->DeleteConnection();
    impl_->handshake_done_.store(false);
  }
  impl_->CloseSerial();
  impl_->connected_ = false;
  impl_->control_mode_written_ = false;
  impl_->last_sent_commands_.clear();
  impl_->last_command_time_ = std::chrono::steady_clock::time_point{};
}

bool ZqwlCanfdDriver::Impl::OpenSerial() {
  serial_fd_ = open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("zqwl_canfd"),
                 "OpenSerial FAILED path=%s errno=%d %s", device_path_.c_str(),
                 errno, strerror(errno));
    return false;
  }

  struct termios tty;
  std::memset(&tty, 0, sizeof(tty));
  if (tcgetattr(serial_fd_, &tty) != 0) {
    CloseSerial();
    return false;
  }

  cfmakeraw(&tty);
  const speed_t serial_speed = map_baud_rate(serial_baud_rate_);
  cfsetispeed(&tty, serial_speed);
  cfsetospeed(&tty, serial_speed);
  tty.c_cflag |= CREAD | CLOCAL;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;  // 100 ms timeout

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    CloseSerial();
    return false;
  }
  tcflush(serial_fd_, TCIOFLUSH);
  return true;
}

void ZqwlCanfdDriver::Impl::CloseSerial() {
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool ZqwlCanfdDriver::Impl::SendConfigFrame(uint8_t func_code,
                                            const uint8_t* data) {
  if (serial_fd_ < 0) return false;

  uint8_t frame[22] = {0};
  frame[0] = 0x49;
  frame[1] = 0x3B;
  frame[2] = func_code;
  frame[3] = 0x57;  // write
  if (data) {
    std::memcpy(&frame[4], data, 16);
  }
  frame[20] = 0x45;
  frame[21] = 0x2E;

  size_t total = 0;
  while (total < sizeof(frame)) {
    const ssize_t written =
        ::write(serial_fd_, frame + total, sizeof(frame) - total);
    if (written < 0) {
      if (errno == EINTR || errno == EAGAIN) continue;
      return false;
    }
    if (written == 0) return false;
    total += static_cast<size_t>(written);
  }
  return true;
}

bool ZqwlCanfdDriver::Impl::SetCanParameters() {
  // GHand-Cpp-SDK uses the ZQWL adapter's standard bitrate-code table.
  const auto arbitration = common_bitrate_codes().find(can_bitrate_);
  const auto data_phase = common_data_bitrate_codes().find(dbit_rate_);
  if (arbitration == common_bitrate_codes().end() ||
      data_phase == common_data_bitrate_codes().end()) {
    RCLCPP_ERROR(rclcpp::get_logger("zqwl_canfd"),
                 "Unsupported CANFD bitrate pair: %u/%u", can_bitrate_,
                 dbit_rate_);
    return false;
  }

  uint8_t data[16] = {0};
  data[0] = can_channel_;
  data[1] = 0x00;  // use standard bitrate codes
  data[2] = static_cast<uint8_t>((arbitration->second << 4) |
                                 data_phase->second);
  return SendConfigFrame(0x42, data);
}

bool ZqwlCanfdDriver::Impl::StartCanChannel() {
  uint8_t data[16] = {0};
  data[0] = 0x01;  // parameter effective
  if (can_channel_ == 0) {
    data[2] = 0x01;
  } else if (can_channel_ == 1) {
    data[3] = 0x01;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("zqwl_canfd"),
                 "ZQWL adapter only supports CAN0/CAN1, got channel %u",
                 can_channel_);
    return false;
  }
  return SendConfigFrame(0x44, data);
}

bool ZqwlCanfdDriver::Impl::TransmitFrame(uint32_t can_id, const uint8_t* data,
                                          uint8_t len) {
  if (serial_fd_ < 0) return false;

  const uint8_t data_len = canfd::nearest_valid_canfd_length(len);
  std::vector<uint8_t> frame;
  frame.reserve(8 + data_len);
  frame.push_back(0x5A);

  // GHand-Cpp-SDK/ZQWL stores the actual valid CANFD payload length here,
  // rather than the ISO DLC ordinal (9..15).
  uint8_t byte1 =
      static_cast<uint8_t>((can_channel_ & 0x01) << 7) | (data_len & 0x7F);
  frame.push_back(byte1);

  // Byte2: tx type, channel high bits, frame format, frame type, BRS
  uint8_t byte2 = 0x01;  // BRS enabled
  byte2 |= static_cast<uint8_t>(((can_channel_ >> 1) & 0x03) << 3);
  byte2 |= 0x04;  // extended frame
  frame.push_back(byte2);

  // CAN ID: 4 bytes, MSB has CANFD flag
  frame.push_back(static_cast<uint8_t>((can_id >> 24) & 0xFF) | 0x80);
  frame.push_back(static_cast<uint8_t>((can_id >> 16) & 0xFF));
  frame.push_back(static_cast<uint8_t>((can_id >> 8) & 0xFF));
  frame.push_back(static_cast<uint8_t>(can_id & 0xFF));

  // Data
  if (data != nullptr && len > 0) {
    frame.insert(frame.end(), data, data + len);
  }
  // Pad to valid CANFD length
  if (data_len > len) {
    frame.insert(frame.end(), data_len - len, 0x00);
  }

  frame.push_back(0xA5);

  size_t total = 0;
  while (total < frame.size()) {
    const ssize_t written =
        ::write(serial_fd_, frame.data() + total, frame.size() - total);
    if (written < 0) {
      if (errno == EINTR || errno == EAGAIN) continue;
      RCLCPP_ERROR(rclcpp::get_logger("zqwl_canfd"),
                   "CANFD serial write failed: errno=%d %s", errno,
                   strerror(errno));
      return false;
    }
    if (written == 0) return false;
    total += static_cast<size_t>(written);
  }
  return true;
}

bool ZqwlCanfdDriver::Impl::WriteHoldingRegister(uint16_t addr,
                                                 uint16_t value) {
  const std::vector<uint8_t> data = {
      static_cast<uint8_t>((value >> 8) & 0xFF),
      static_cast<uint8_t>(value & 0xFF)};
  return WriteRegisters(addr, data);
}

bool ZqwlCanfdDriver::Impl::WriteRegisters(
    uint16_t addr, const std::vector<uint8_t>& data, uint32_t timeout_ms) {
  if (data.empty() || (data.size() % 2) != 0) return false;
  if (timeout_ms == 0) timeout_ms = response_timeout_ms_;

  std::lock_guard<std::mutex> transaction_lock(io_mutex_);
  uint8_t function_code = canfd::modbus::kWriteMultipleRegisters;
  std::vector<uint8_t> payload;
  if (data.size() == 2) {
    function_code = canfd::modbus::kWriteSingleRegister;
    payload = canfd::encode_write_single_register(
        addr, static_cast<uint16_t>((data[0] << 8) | data[1]));
  } else {
    payload = canfd::encode_write_multiple_registers(addr, data);
  }

  constexpr size_t kSegmentSize = 64;
  const size_t segment_count =
      (payload.size() + kSegmentSize - 1) / kSegmentSize;
  for (size_t segment = 0; segment < segment_count; ++segment) {
    const size_t offset = segment * kSegmentSize;
    const size_t length =
        std::min(kSegmentSize, payload.size() - offset);
    const uint32_t can_id = canfd::pack_modbus_arbitration_id(
        src_id_, dst_id_, 0, function_code, segment == 0,
        segment + 1 == segment_count, segment % 2,
        static_cast<uint8_t>(segment));
    if (!TransmitFrame(can_id, payload.data() + offset,
                       static_cast<uint8_t>(length))) {
      return false;
    }
  }

  std::vector<uint8_t> response;
  return ReadModbusResponse(function_code, response, timeout_ms);
}

bool ZqwlCanfdDriver::Impl::EstablishConnection() {
  const std::vector<uint8_t> payload(kEstablishConnectionPayload.begin(),
                                     kEstablishConnectionPayload.end());
  std::vector<uint8_t> candidate_ids;
  if (preferred_node_id_ == 0x31 || preferred_node_id_ == 0x32) {
    candidate_ids.push_back(preferred_node_id_);
  }
  for (uint8_t candidate : {static_cast<uint8_t>(0x31),
                            static_cast<uint8_t>(0x32)}) {
    if (std::find(candidate_ids.begin(), candidate_ids.end(), candidate) ==
        candidate_ids.end()) {
      candidate_ids.push_back(candidate);
    }
  }

  for (uint8_t dst : candidate_ids) {
    dst_id_ = dst;
    const uint32_t can_id = canfd::pack_modbus_arbitration_id(
        src_id_, dst_id_, 0, 0x02, 1, 1, 0, 0);
    uint8_t exception_code = 0;
    {
      std::lock_guard<std::mutex> transaction_lock(io_mutex_);
      if (!TransmitFrame(can_id, payload.data(),
                         static_cast<uint8_t>(payload.size()))) {
        continue;
      }

      std::vector<uint8_t> response;
      if (ReadModbusResponse(0x02, response, 500, &exception_code)) {
        return true;
      }
    }

    // SDK treats Modbus exception 0x03 as a possible existing session and
    // verifies the node by reading the device-name register.
    if (exception_code == 0x03) {
      std::vector<uint8_t> device_name;
      if (ReadInputRegisters(0x1000, 1, device_name)) return true;
    }
  }
  return false;
}

void ZqwlCanfdDriver::Impl::DeleteConnection() {
  std::lock_guard<std::mutex> transaction_lock(io_mutex_);
  const std::vector<uint8_t> payload(kDeleteConnectionPayload.begin(),
                                     kDeleteConnectionPayload.end());
  const uint32_t can_id =
      canfd::pack_modbus_arbitration_id(src_id_, dst_id_, 0, 0x05, 1, 1, 0, 0);
  TransmitFrame(can_id, payload.data(), static_cast<uint8_t>(payload.size()));
}

bool ZqwlCanfdDriver::Impl::ReadInputRegisters(uint16_t addr, uint16_t count,
                                               std::vector<uint8_t>& out) {
  std::lock_guard<std::mutex> transaction_lock(io_mutex_);
  const std::vector<uint8_t> payload = {
      static_cast<uint8_t>((addr >> 8) & 0xFF),
      static_cast<uint8_t>(addr & 0xFF),
      static_cast<uint8_t>((count >> 8) & 0xFF),
      static_cast<uint8_t>(count & 0xFF)};

  const uint32_t can_id =
      canfd::pack_modbus_arbitration_id(src_id_, dst_id_, 0, 0x04, 1, 1, 0, 0);

  if (!TransmitFrame(can_id, payload.data(),
                     static_cast<uint8_t>(payload.size()))) {
    return false;
  }

  return ReadModbusResponse(0x04, out, response_timeout_ms_);
}

bool ZqwlCanfdDriver::Impl::ReadModbusResponse(uint8_t expected_func,
                                               std::vector<uint8_t>& out,
                                               uint32_t timeout_ms,
                                               uint8_t* exception_code) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  std::map<uint8_t, std::vector<uint8_t>> segments;

  while (std::chrono::steady_clock::now() < deadline) {
    canfd::Frame frame{};
    if (!ReadNextCanFrame(&frame, deadline)) {
      return false;
    }

    canfd::ModbusArbitrationId arb(frame.id);
    if (arb.ack() != 1 || arb.dst_id() != src_id_ || arb.src_id() != dst_id_) {
      continue;
    }

    if (arb.func_code() == (0x80 | expected_func)) {
      if (exception_code != nullptr && frame.len > 0) {
        *exception_code = frame.data[0];
      }
      return false;
    }
    if (arb.func_code() != expected_func) {
      continue;
    }

    size_t data_offset = 0;
    if (arb.seg_num() == 0 && frame.len > 0) {
      data_offset = 1;  // strip Modbus byte-count prefix
    }
    segments[arb.seg_num()].assign(frame.data + data_offset,
                                   frame.data + frame.len);

    if (arb.end() == 1) {
      out.clear();
      for (const auto& kv : segments) {
        out.insert(out.end(), kv.second.begin(), kv.second.end());
      }
      return true;
    }
  }

  return false;
}

bool ZqwlCanfdDriver::Impl::ReadNextCanFrame(
    canfd::Frame* out, std::chrono::steady_clock::time_point deadline) {
  while (std::chrono::steady_clock::now() < deadline) {
    // During normal operation the receive thread should exit promptly when
    // asked to stop; during handshake run_thread_ is not yet running.
    if (handshake_done_.load() && !run_thread_.load()) {
      return false;
    }

    if (serial_fd_ < 0) {
      return false;
    }

    uint8_t byte = 0;
    const ssize_t n = ::read(serial_fd_, &byte, 1);
    if (n <= 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    rx_buffer_.push_back(byte);

    // Trim until we see a known start byte.
    while (rx_parse_offset_ < rx_buffer_.size() &&
           rx_buffer_[rx_parse_offset_] != 0x49 &&
           rx_buffer_[rx_parse_offset_] != 0x5A) {
      ++rx_parse_offset_;
    }
    if (rx_parse_offset_ >= rx_buffer_.size()) continue;

    // Config response: 0x49 0x3B ... 0x45 0x2E, fixed 22 bytes.
    if (rx_buffer_[rx_parse_offset_] == 0x49 &&
        rx_parse_offset_ + 1 < rx_buffer_.size() &&
        rx_buffer_[rx_parse_offset_ + 1] == 0x3B) {
      if (rx_buffer_.size() - rx_parse_offset_ >= 22) {
        rx_parse_offset_ += 22;
      }
      continue;
    }

    // Heartbeat or CAN data: starts with 0x5A, ends with 0xA5.
    if (rx_buffer_[rx_parse_offset_] == 0x5A) {
      if (rx_parse_offset_ + 1 >= rx_buffer_.size()) continue;

      const uint8_t byte1 = rx_buffer_[rx_parse_offset_ + 1];
      size_t data_len = 0;
      size_t expected_len = 0;
      if (byte1 == 0xFF) {
        data_len = 9;
        expected_len = 17;
      } else if (byte1 == 0xFE) {
        data_len = 24;
        expected_len = 32;
      } else {
        const int decoded_length =
            payload_length_from_adapter_dlc(byte1 & 0x7F);
        if (decoded_length < 0) {
          ++rx_parse_offset_;
          continue;
        }
        data_len = static_cast<size_t>(decoded_length);
        expected_len = 8 + data_len;
      }

      if (rx_buffer_.size() - rx_parse_offset_ >= expected_len) {
        if (rx_buffer_[rx_parse_offset_ + expected_len - 1] == 0xA5) {
          // The adapter heartbeat is informational.  Match the official
          // GHand C++ SDK and ignore it instead of interpreting adapter bytes
          // as a CAN bus state and disconnecting a healthy hand.
          if (byte1 != 0xFF && byte1 != 0xFE) {
            out->id =
                (static_cast<uint32_t>(rx_buffer_[rx_parse_offset_ + 3] & 0x7F)
                 << 24) |
                (static_cast<uint32_t>(rx_buffer_[rx_parse_offset_ + 4])
                 << 16) |
                (static_cast<uint32_t>(rx_buffer_[rx_parse_offset_ + 5]) << 8) |
                static_cast<uint32_t>(rx_buffer_[rx_parse_offset_ + 6]);
            out->len = static_cast<uint8_t>(data_len);
            if (out->len > 64) out->len = 64;
            std::memcpy(out->data, rx_buffer_.data() + rx_parse_offset_ + 7,
                        out->len);
            out->is_fd = (rx_buffer_[rx_parse_offset_ + 3] & 0x80) != 0;
            out->is_extended =
                (rx_buffer_[rx_parse_offset_ + 2] & 0x04) != 0;
            rx_parse_offset_ += expected_len;
            return true;
          }
        }
        rx_parse_offset_ += expected_len;
      }

      if (rx_buffer_.size() > 512) {
        rx_buffer_.clear();
        rx_parse_offset_ = 0;
      }
    }

    if (rx_parse_offset_ >= 128 || rx_parse_offset_ >= rx_buffer_.size()) {
      if (rx_parse_offset_ < rx_buffer_.size()) {
        rx_buffer_.erase(rx_buffer_.begin(),
                         rx_buffer_.begin() + rx_parse_offset_);
      } else {
        rx_buffer_.clear();
      }
      rx_parse_offset_ = 0;
    }
  }

  return false;
}

void ZqwlCanfdDriver::Impl::ReceiveLoop() {
  static constexpr auto kPollInterval = std::chrono::milliseconds(10);
  auto last_tactile_poll = std::chrono::steady_clock::time_point{};

  while (run_thread_.load()) {
    if (serial_fd_ < 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    PollJoints();
    PollHandState();
    const auto now = std::chrono::steady_clock::now();
    if (!tactile_regions_.empty() &&
        (last_tactile_poll.time_since_epoch().count() == 0 ||
         now - last_tactile_poll >= std::chrono::milliseconds(100))) {
      PollTactile();
      last_tactile_poll = now;
    }

    std::this_thread::sleep_for(kPollInterval);
  }
}

void ZqwlCanfdDriver::Impl::PollHandState() {
  std::vector<uint8_t> payload;
  if (!ReadInputRegisters(0x1047, 11, payload)) {
    return;
  }
  if (payload.size() < 22) {
    return;
  }

  uint16_t raw[11] = {0};
  for (size_t i = 0; i < 11; ++i) {
    raw[i] = static_cast<uint16_t>((payload[i * 2] << 8) | payload[i * 2 + 1]);
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_data_.hand_state = parse_hand_info(raw, 11);
  latest_data_.valid = true;
  last_rx_time_ = std::chrono::steady_clock::now();
  rx_timeout_initialized_ = true;
}

void ZqwlCanfdDriver::Impl::PollJoints() {
  if (valid_joints_.empty()) {
    return;
  }

  uint8_t max_id = 0;
  for (const auto id : valid_joints_) {
    max_id = std::max(max_id, static_cast<uint8_t>(id));
  }
  const uint16_t count = static_cast<uint16_t>((max_id + 1) * 5);

  std::vector<uint8_t> payload;
  if (!ReadInputRegisters(0x1023, count, payload)) {
    return;
  }
  if (payload.size() < count * 2) {
    return;
  }

  std::vector<uint16_t> raw(count);
  for (uint16_t i = 0; i < count; ++i) {
    raw[i] = static_cast<uint16_t>((payload[i * 2] << 8) | payload[i * 2 + 1]);
  }

  auto joints = parse_joints(raw.data(), count, valid_joints_);

  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_data_.joints = std::move(joints);
  latest_data_.valid = true;
  last_rx_time_ = std::chrono::steady_clock::now();
  rx_timeout_initialized_ = true;
}

void ZqwlCanfdDriver::Impl::PollTactile() {
  if (tactile_regions_.empty()) {
    return;
  }

  const size_t region_count = tactile_regions_.size();
  // The SDK first reads the 16-register tactile status/resultant block, then
  // reads each region's distributed samples separately.
  std::vector<uint8_t> status_payload;
  if (!ReadInputRegisters(0x1110, 16, status_payload)) {
    return;
  }
  if (status_payload.size() < 32 || 1 + region_count * 3 > 16) {
    return;
  }

  std::vector<uint16_t> regs(16, 0);
  for (size_t i = 0; i < regs.size(); ++i) {
    if (2 * i + 1 < status_payload.size()) {
      regs[i] =
          static_cast<uint16_t>((status_payload[2 * i] << 8) |
                                status_payload[2 * i + 1]);
    }
  }

  std::vector<TactileRegion> regions;
  regions.reserve(region_count);
  uint16_t current_addr = 0x1110 + 16;
  for (size_t i = 0; i < region_count; ++i) {
    TactileRegion tr;
    tr.name = tactile_regions_[i].first;
    tr.resultant_force =
        parse_tactile_resultant(regs.data(), static_cast<int>(i));

    const size_t sensor_count = tactile_regions_[i].second;
    if (sensor_count > 0) {
      const uint16_t distributed_registers =
          static_cast<uint16_t>((sensor_count * 3 + 1) / 2);
      std::vector<uint8_t> distributed_payload;
      if (!ReadInputRegisters(current_addr, distributed_registers,
                              distributed_payload)) {
        return;
      }
      tr.distributed_forces = parse_tactile_distributed(
          distributed_payload.data(), distributed_payload.size(),
          static_cast<int>(sensor_count));
      current_addr = static_cast<uint16_t>(current_addr +
                                           distributed_registers);
    }
    regions.push_back(std::move(tr));
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_data_.tactile_regions = std::move(regions);
  latest_data_.valid = true;
  last_rx_time_ = std::chrono::steady_clock::now();
  rx_timeout_initialized_ = true;
}

}  // namespace ghand_comm
