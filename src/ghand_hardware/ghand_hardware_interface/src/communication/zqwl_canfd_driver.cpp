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
#include <tuple>
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

struct CanTiming {
  uint8_t sjw = 0;
  uint8_t tseg1 = 0;
  uint8_t tseg2 = 0;
  uint16_t brp = 1;
};

bool compute_zqwl_timing(uint32_t target_bitrate, CanTiming* out) {
  // ZQWL formula: Baud = 60000000 / ((SJW+1 + TSEG1+1 + TSEG2+1) * BRP)
  // where SJW/TSEG1/TSEG2 are REGISTER values in [0,3]/[0,15]/[0,7].
  // Constraint: total_tq = SJW + TSEG1 + TSEG2 + 3
  // Sample point = (SJW + TSEG1 + 2) / total_tq, target ≈ 75%
  constexpr uint32_t kCanClock = 60000000;
  constexpr uint16_t kMaxBrp = 0x0400;
  constexpr uint8_t kMaxTseg1 = 0x0F;
  constexpr uint8_t kMaxTseg2 = 0x07;

  if (target_bitrate == 0) return false;

  for (uint16_t brp = 1; brp <= kMaxBrp; ++brp) {
    const uint32_t tq_hz = kCanClock / brp;
    if (tq_hz == 0) continue;
    const uint32_t total_tq = tq_hz / target_bitrate;
    if (total_tq < 4 || total_tq > (kMaxTseg1 + kMaxTseg2 + 3 + 3)) continue;
    if (tq_hz / total_tq != target_bitrate) continue;

    for (uint8_t sjw = 0; sjw <= 3; ++sjw) {
      // tseg1 + sjw + 2 ≈ 0.75 * total_tq  →  tseg1 ≈ 0.75*total_tq - sjw - 2
      int tseg1 = static_cast<int>((total_tq * 3) / 4) - sjw - 2;
      if (tseg1 < 0) continue;
      if (tseg1 > static_cast<int>(kMaxTseg1)) continue;

      // total_tq = sjw + tseg1 + tseg2 + 3  →  tseg2 = total_tq - 3 - sjw -
      // tseg1
      int tseg2 = static_cast<int>(total_tq) - 3 - sjw - tseg1;
      if (tseg2 < 0) continue;
      if (tseg2 > static_cast<int>(kMaxTseg2)) continue;

      // Verify sample point in reasonable range (70-80%)
      const int sp = ((sjw + 1 + tseg1 + 1) * 100) / static_cast<int>(total_tq);
      if (sp < 70 || sp > 80) continue;

      out->brp = brp;
      out->tseg1 = static_cast<uint8_t>(tseg1);
      out->tseg2 = static_cast<uint8_t>(tseg2);
      out->sjw = sjw;
      return true;
    }
  }
  return false;
}

}  // namespace

// Connection-establishment payload from the reference Python SDK.
constexpr std::array<uint8_t, 6> kEstablishConnectionPayload = {
    0x00, 0x31, 0x00, 0x01, 0x00, 0x00};  // NOLINT(whitespace/indent_namespace)
constexpr std::array<uint8_t, 8> kDeleteConnectionPayload = {
    0x00, 0x30, 0x00, 0x02,
    0x00, 0x00, 0x00, 0x00};  // NOLINT(whitespace/indent_namespace)
constexpr uint8_t kDefaultSrcId = 0x0A;
constexpr uint8_t kDefaultDstId = 0x31;

struct ZqwlCanfdDriver::Impl {
  std::string device_path_ = "/dev/ttyACM0";
  uint32_t serial_baud_rate_ = 2000000;
  uint32_t can_bitrate_ = 1000000;
  uint32_t dbit_rate_ = 5000000;
  uint8_t can_channel_ = 0;
  uint8_t device_id_ = 0x32;
  uint8_t src_id_ = kDefaultSrcId;
  uint8_t dst_id_ = kDefaultDstId;
  CtrlMode control_mode_ = CtrlMode::POSITION;

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

  // Persistent receive buffer shared by handshake and polling reads.
  std::vector<uint8_t> rx_buffer_;
  size_t rx_parse_offset_ = 0;

  bool OpenSerial();
  void CloseSerial();
  bool SendConfigFrame(uint8_t func_code, const uint8_t* data);
  bool SetCanParameters();
  bool StartCanChannel();
  bool TransmitFrame(uint32_t can_id, const uint8_t* data, uint8_t len);
  bool WriteHoldingRegister(uint16_t addr, uint16_t value);

  bool EstablishConnection();
  void DeleteConnection();
  bool ReadInputRegisters(uint16_t addr, uint16_t count,
                          std::vector<uint8_t>& out);
  bool ReadModbusResponse(uint8_t expected_func, std::vector<uint8_t>& out,
                          uint32_t timeout_ms);
  bool ReadNextCanFrame(canfd::Frame* out,
                        std::chrono::steady_clock::time_point deadline);

  void ReceiveLoop();
  void PollHandState();
  void PollJoints();
  void PollTactile();
  void ProcessHeartbeat(const std::vector<uint8_t>& frame);
};

ZqwlCanfdDriver::ZqwlCanfdDriver() : impl_(std::make_unique<Impl>()) {}

ZqwlCanfdDriver::~ZqwlCanfdDriver() { close(); }

const char* ZqwlCanfdDriver::get_name() const { return "zqwl_canfd"; }

bool ZqwlCanfdDriver::init(
    const std::unordered_map<std::string, std::string>& params) {
  auto it = params.find("device");
  if (it != params.end() && !it->second.empty()) {
    impl_->device_path_ = it->second;
  }

  impl_->serial_baud_rate_ = static_cast<uint32_t>(parse_int_default(
      params.count("baud_rate") ? params.at("baud_rate") : "", 2000000));
  impl_->can_bitrate_ = static_cast<uint32_t>(parse_int_default(
      params.count("bitrate") ? params.at("bitrate") : "", 1000000));
  impl_->dbit_rate_ = static_cast<uint32_t>(
      parse_int_default(params.count("data_rate")   ? params.at("data_rate")
                        : params.count("dbit_rate") ? params.at("dbit_rate")
                                                    : "",
                        5000000));
  impl_->can_channel_ = static_cast<uint8_t>(parse_int_default(
      params.count("can_channel") ? params.at("can_channel") : "", 0));
  impl_->device_id_ = static_cast<uint8_t>(
      parse_int_default(params.count("node_id")     ? params.at("node_id")
                        : params.count("device_id") ? params.at("device_id")
                                                    : "",
                        50));

  auto cm = params.find("control_mode");
  if (cm != params.end()) {
    if (cm->second == "torque" || cm->second == "1") {
      impl_->control_mode_ = CtrlMode::TORQUE;
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

  // Try multiple device paths (ZQWL can re-enumerate as ttyACM0 or ttyACM1).
  {
    std::vector<std::string> paths = {impl_->device_path_, "/dev/ttyACM0",
                                      "/dev/ttyACM1", "/dev/ttyUSB0"};
    bool opened = false;
    for (const auto& p : paths) {
      if (p.empty()) continue;
      impl_->device_path_ = p;
      if (impl_->OpenSerial()) {
        opened = true;
        break;
      }
    }
    if (!opened) return false;
  }

  if (!impl_->SetCanParameters()) {
    impl_->CloseSerial();
    return false;
  }

  // Small delay for device to apply parameters.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  if (!impl_->StartCanChannel()) {
    impl_->CloseSerial();
    return false;
  }

  // ZQWL devices reset their USB interface after the CAN channel is enabled.
  // Close the current serial handle, wait for re-enumeration, then reopen.
  impl_->CloseSerial();
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  std::vector<std::string> candidates = {impl_->device_path_, "/dev/ttyACM0",
                                         "/dev/ttyACM1", "/dev/ttyUSB0"};
  bool reopened = false;
  for (const auto& path : candidates) {
    if (path.empty()) continue;
    impl_->device_path_ = path;
    if (impl_->OpenSerial()) {
      reopened = true;
      break;
    }
  }
  if (!reopened) {
    return false;
  }

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
      "initialized: device=%s baud_rate=%u data_rate=%u node_id=%u channel=%u",
      impl_->device_path_.c_str(), impl_->serial_baud_rate_, impl_->dbit_rate_,
      impl_->device_id_, impl_->can_channel_);

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
  if (impl_->rx_timeout_initialized_) {
    const auto elapsed =
        std::chrono::steady_clock::now() - impl_->last_rx_time_;
    if (elapsed > Impl::kRxTimeout) {
      impl_->connected_.store(false);
      return false;
    }
  }

  if (!impl_->connected_.load() || commands.empty()) {
    return true;
  }

  // Write control mode before joint commands.
  const uint16_t mode_value =
      static_cast<uint16_t>(static_cast<uint8_t>(impl_->control_mode_) << 8);
  if (!impl_->WriteHoldingRegister(0x0010, mode_value)) {
    return false;
  }

  for (const auto& cmd : commands) {
    const auto& reg_map = holding_register_map();
    auto it = reg_map.find(cmd.id);
    if (it == reg_map.end()) continue;

    const auto regs = encode_joint_command(cmd);
    const std::vector<uint8_t> data = {
        static_cast<uint8_t>((regs.first >> 8) & 0xFF),
        static_cast<uint8_t>(regs.first & 0xFF),
        static_cast<uint8_t>((regs.second >> 8) & 0xFF),
        static_cast<uint8_t>(regs.second & 0xFF)};

    const auto payload =
        canfd::encode_write_multiple_registers(it->second, data);
    const uint32_t can_id = canfd::pack_modbus_arbitration_id(
        impl_->src_id_, impl_->dst_id_, 0,
        canfd::modbus::kWriteMultipleRegisters, 1, 1, 0, 0);

    if (!impl_->TransmitFrame(can_id, payload.data(),
                              static_cast<uint8_t>(payload.size()))) {
      return false;
    }
  }
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
}

bool ZqwlCanfdDriver::Impl::OpenSerial() {
  serial_fd_ = open(device_path_.c_str(), O_RDWR | O_NOCTTY);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("zqwl_canfd"),
                 "OpenSerial FAILED path=%s errno=%d %s", device_path_.c_str(),
                 errno, strerror(errno));
    return false;
  }

  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0) {
    CloseSerial();
    return false;
  }

  cfsetispeed(&tty, map_baud_rate(serial_baud_rate_));
  cfsetospeed(&tty, map_baud_rate(serial_baud_rate_));

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit
  tty.c_cflag &= ~PARENB;                      // no parity
  tty.c_cflag &= ~CSTOPB;                      // 1 stop bit
  tty.c_cflag &= ~CRTSCTS;                     // no flow control
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_oflag &= ~OPOST;

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

  const ssize_t written = ::write(serial_fd_, frame, sizeof(frame));
  return written == static_cast<ssize_t>(sizeof(frame));
}

bool ZqwlCanfdDriver::Impl::SetCanParameters() {
  // Use timing values from the reference BrainCo hand project.
  // Arbitration: 1.154 MHz, sample point ~77%
  // Data phase:  4.615 MHz, sample point ~77%
  uint8_t data[16] = {0};
  data[0] = can_channel_;  // CAN channel
  data[1] = 0x01;          // use custom baud rate
  data[2] = 1;             // abit.sjw
  data[3] = 7;             // abit.tseg1
  data[4] = 2;             // abit.tseg2
  data[5] = 0;             // abit.brp high
  data[6] = 4;             // abit.brp low (= 4)
  data[7] = 1;             // dbit.sjw
  data[8] = 7;             // dbit.tseg1
  data[9] = 2;             // dbit.tseg2
  data[10] = 0;            // dbit.brp high
  data[11] = 1;            // dbit.brp low (= 1)
  // data[12..15] reserved
  return SendConfigFrame(0x42, data);
}

bool ZqwlCanfdDriver::Impl::StartCanChannel() {
  uint8_t data[16] = {0};
  data[0] = 0x01;                               // parameter effective
  data[1] = (can_channel_ == 0) ? 0x01 : 0x00;  // CAN0 on
  data[2] = (can_channel_ == 1) ? 0x01 : 0x00;  // CAN1 on
  // data[3..15] reserved
  return SendConfigFrame(0x44, data);
}

bool ZqwlCanfdDriver::Impl::TransmitFrame(uint32_t can_id, const uint8_t* data,
                                          uint8_t len) {
  if (serial_fd_ < 0) return false;

  const uint8_t data_len = canfd::nearest_valid_canfd_length(len);
  const uint8_t dlc = canfd::canfd_length_to_dlc(data_len);
  if (dlc > 15) return false;

  std::vector<uint8_t> frame;
  frame.reserve(8 + data_len);
  frame.push_back(0x5A);

  // Byte1: channel high bit + DLC code
  uint8_t byte1 =
      static_cast<uint8_t>((can_channel_ & 0x01) << 7) | (dlc & 0x7F);
  frame.push_back(byte1);

  // Byte2: tx type, channel high bits, frame format, frame type, BRS
  uint8_t byte2 = 0x00;
  byte2 |= static_cast<uint8_t>((can_channel_ & 0x06) << 3);  // bits 4:3
  byte2 |= 0x08;                                              // extended frame
  byte2 |= 0x10;                                              // CANFD frame
  byte2 |= 0x02;                                              // CANFD BRS
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

  std::lock_guard<std::mutex> lock(io_mutex_);
  const ssize_t written = ::write(serial_fd_, frame.data(), frame.size());
  if (written != static_cast<ssize_t>(frame.size())) {
    RCLCPP_ERROR(
        rclcpp::get_logger("zqwl_canfd"),
        "TransmitFrame failed: fd=%d expected=%zu written=%zd errno=%d",
        serial_fd_, frame.size(), written, errno);
    return false;
  }
  return true;
}

bool ZqwlCanfdDriver::Impl::WriteHoldingRegister(uint16_t addr,
                                                 uint16_t value) {
  const auto payload = canfd::encode_write_single_register(addr, value);
  const uint32_t can_id = canfd::pack_modbus_arbitration_id(
      src_id_, dst_id_, 0, canfd::modbus::kWriteSingleRegister, 1, 1, 0, 0);
  return TransmitFrame(can_id, payload.data(),
                       static_cast<uint8_t>(payload.size()));
}

bool ZqwlCanfdDriver::Impl::EstablishConnection() {
  const std::vector<uint8_t> payload(kEstablishConnectionPayload.begin(),
                                     kEstablishConnectionPayload.end());
  const std::vector<uint8_t> candidate_ids = {0x31, 0x32};

  for (uint8_t dst : candidate_ids) {
    dst_id_ = dst;
    const uint32_t can_id = canfd::pack_modbus_arbitration_id(
        src_id_, dst_id_, 0, 0x02, 1, 1, 0, 0);
    if (!TransmitFrame(can_id, payload.data(),
                       static_cast<uint8_t>(payload.size()))) {
      continue;
    }

    std::vector<uint8_t> response;
    if (ReadModbusResponse(0x02, response, 500)) {
      return true;
    }
  }
  return false;
}

void ZqwlCanfdDriver::Impl::DeleteConnection() {
  const std::vector<uint8_t> payload(kDeleteConnectionPayload.begin(),
                                     kDeleteConnectionPayload.end());
  const uint32_t can_id =
      canfd::pack_modbus_arbitration_id(src_id_, dst_id_, 0, 0x05, 1, 1, 0, 0);
  TransmitFrame(can_id, payload.data(), static_cast<uint8_t>(payload.size()));
}

bool ZqwlCanfdDriver::Impl::ReadInputRegisters(uint16_t addr, uint16_t count,
                                               std::vector<uint8_t>& out) {
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

  return ReadModbusResponse(0x04, out, 500);
}

bool ZqwlCanfdDriver::Impl::ReadModbusResponse(uint8_t expected_func,
                                               std::vector<uint8_t>& out,
                                               uint32_t timeout_ms) {
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
        data_len = canfd::dlc_to_canfd_length(byte1 & 0x7F);
        expected_len = 8 + data_len;
      }

      if (rx_buffer_.size() - rx_parse_offset_ >= expected_len) {
        if (rx_buffer_[rx_parse_offset_ + expected_len - 1] == 0xA5) {
          if (byte1 == 0xFF || byte1 == 0xFE) {
            std::vector<uint8_t> frame(
                rx_buffer_.begin() + rx_parse_offset_,
                rx_buffer_.begin() + rx_parse_offset_ + expected_len);
            ProcessHeartbeat(frame);
          } else {
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
            out->is_extended = (rx_buffer_[rx_parse_offset_ + 2] & 0x08) != 0;
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

  while (run_thread_.load()) {
    if (serial_fd_ < 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    PollHandState();
    PollJoints();
    if (!tactile_regions_.empty()) {
      PollTactile();
    }

    std::this_thread::sleep_for(kPollInterval);
  }
}

void ZqwlCanfdDriver::Impl::PollHandState() {
  std::vector<uint8_t> payload;
  if (!ReadInputRegisters(0x1021, 2, payload)) {
    return;
  }
  if (payload.size() < 4) {
    return;
  }

  const uint16_t raw[2] = {
      static_cast<uint16_t>((payload[0] << 8) | payload[1]),
      static_cast<uint16_t>((payload[2] << 8) | payload[3])};

  const auto info = parse_hand_info(raw, 2);

  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_data_.hand_state.state = std::get<0>(info);
  latest_data_.hand_state.error = std::get<1>(info);
  latest_data_.hand_state.temperature = std::get<2>(info);
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
  const uint16_t count = static_cast<uint16_t>((max_id + 1) * 3);

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
  size_t distributed_bytes = 0;
  for (const auto& region : tactile_regions_) {
    distributed_bytes += region.second * 3;
  }
  const size_t total_bytes = 2 + region_count * 6 + distributed_bytes;
  const uint16_t count = static_cast<uint16_t>((total_bytes + 1) / 2);

  std::vector<uint8_t> payload;
  if (!ReadInputRegisters(0x1080, count, payload)) {
    return;
  }
  if (payload.size() < 2 + region_count * 6) {
    return;
  }

  std::vector<uint16_t> regs(count, 0);
  for (size_t i = 0; i < count; ++i) {
    if (2 * i + 1 < payload.size()) {
      regs[i] =
          static_cast<uint16_t>((payload[2 * i] << 8) | payload[2 * i + 1]);
    }
  }

  std::vector<TactileRegion> regions;
  regions.reserve(region_count);
  for (size_t i = 0; i < region_count; ++i) {
    TactileRegion tr;
    tr.name = tactile_regions_[i].first;
    tr.resultant_force =
        parse_tactile_resultant(regs.data(), static_cast<int>(i));

    const size_t sensor_count = tactile_regions_[i].second;
    if (sensor_count > 0) {
      const size_t dist_off = 2 + region_count * 6;
      size_t dist_start = dist_off;
      for (size_t j = 0; j < i; ++j) {
        dist_start += tactile_regions_[j].second * 3;
      }
      const size_t dist_len = sensor_count * 3;
      if (dist_start + dist_len <= payload.size()) {
        tr.distributed_forces =
            parse_tactile_distributed(payload.data() + dist_start, dist_len,
                                      static_cast<int>(sensor_count));
      }
    }
    regions.push_back(std::move(tr));
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_data_.tactile_regions = std::move(regions);
  latest_data_.valid = true;
  last_rx_time_ = std::chrono::steady_clock::now();
  rx_timeout_initialized_ = true;
}

void ZqwlCanfdDriver::Impl::ProcessHeartbeat(
    const std::vector<uint8_t>& frame) {
  if (frame.size() < 4) return;
  const uint8_t status_byte = frame[3];
  const uint8_t bus_status = status_byte & 0x03;
  if (bus_status == 0x03) {
    connected_.store(false);
  }
}

}  // namespace ghand_comm
