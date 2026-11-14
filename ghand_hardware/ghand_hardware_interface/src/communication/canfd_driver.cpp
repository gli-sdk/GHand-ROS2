// Copyright (c) 2026 GHand ROS2 Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "ghand_hardware_interface/communication/canfd_driver.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
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

#ifdef GHAND_HAS_ZLG_CAN
#include <zcan.h>
#endif

namespace ghand_comm {

namespace {

#ifdef GHAND_HAS_ZLG_CAN

// VCI device type numbers for the libusbcanfd/libzuds API.
// These differ from the zlgcan.h constants used by libzlgcan.so.
int resolve_vci_device_type(const std::string& name) {
  if (name.empty()) return 0;

  if (std::isdigit(static_cast<unsigned char>(name[0]))) {
    return parse_int_default(name, 0);
  }

  static const std::map<std::string, int> kNameToType = {
      {"ZCAN_USBCANFD_100U", 42},
  };

  auto it = kNameToType.find(name);
  return (it != kNameToType.end()) ? it->second : 0;
}

// Compute timing parameters for a desired bitrate given a CAN clock.
// Returns true if a valid (tseg1, tseg2, sjw, brp) combination is found.
bool compute_can_timing(uint32_t clock_hz, uint32_t target_bitrate,
                        uint8_t* tseg1, uint8_t* tseg2, uint8_t* sjw,
                        uint16_t* brp) {
  if (clock_hz == 0 || target_bitrate == 0) return false;

  // Valid ranges from ZCAN_INIT struct.
  const uint16_t kMaxBrp = 512;
  const uint8_t kMaxTseg = 32;

  for (uint16_t p = 1; p <= kMaxBrp; ++p) {
    const uint32_t tq_hz = clock_hz / p;
    if (tq_hz == 0) continue;
    const uint32_t total_tq = tq_hz / target_bitrate;
    if (total_tq == 0 || total_tq > kMaxTseg + kMaxTseg) continue;

    // Try a sample point around 75-80%: tseg1 = 0.75 * total_tq, tseg2 = rest.
    for (uint8_t ts1 = 1; ts1 + 2 <= total_tq && ts1 <= kMaxTseg; ++ts1) {
      const uint8_t ts2 = static_cast<uint8_t>(total_tq - ts1 - 1);
      if (ts2 < 1 || ts2 > kMaxTseg) continue;

      // Verify exact match.
      if (tq_hz / total_tq != target_bitrate) continue;

      *brp = p;
      *tseg1 = ts1;
      *tseg2 = ts2;
      *sjw = std::min<uint8_t>(2, ts2);
      return true;
    }
  }
  return false;
}

#endif  // GHAND_HAS_ZLG_CAN

}  // namespace

// Default CAN arbitration parameters for the ZLG USB-CANFD library.
constexpr uint32_t kDefaultBitrate = 1000000;
constexpr uint32_t kDefaultDataBitrate = 5000000;
constexpr uint32_t kDefaultCanClock = 60000000;

// Default source/destination IDs used in the CANFD Modbus-like protocol.
constexpr uint8_t kDefaultSrcId = 0x0A;
constexpr uint8_t kDefaultDstId = 0x31;

struct CanfdDriver::Impl {
#ifdef GHAND_HAS_ZLG_CAN
  uint32_t device_type_ = 0;
  uint32_t device_index_ = 0;
  uint32_t can_index_ = 0;
  bool device_opened_ = false;

  bool WriteHoldingRegister(uint16_t addr, uint16_t value);
  bool WriteHoldingRegisters(uint16_t addr, const std::vector<uint8_t>& data);
  bool TransmitFrame(uint32_t can_id, const uint8_t* data, uint8_t len);
#endif

  std::atomic<bool> connected_{false};
  std::atomic<bool> run_thread_{false};
  std::thread recv_thread_;

  std::mutex data_mutex_;
  std::mutex io_mutex_;
  CommunicationData latest_data_;

  std::vector<JointId> valid_joints_;
  std::vector<std::pair<std::string, size_t>> tactile_regions_;
  uint8_t device_id_filter_ = 0;
  uint8_t src_id_ = kDefaultSrcId;
  uint8_t dst_id_ = kDefaultDstId;

  canfd::PacketAssembler assembler_;

  void ReceiveLoop();
  void ParseReport(const std::vector<uint8_t>& payload);
  void ParseJointReport(const std::vector<uint8_t>& payload);
  void ParseTactileReport(const std::vector<uint8_t>& payload);
};

CanfdDriver::CanfdDriver() : impl_(std::make_unique<Impl>()) {}

CanfdDriver::~CanfdDriver() { close(); }

const char* CanfdDriver::get_name() const { return "canfd"; }

bool CanfdDriver::init(
    const std::unordered_map<std::string, std::string>& params) {
  const auto mode = parse_hand_control_mode(
      params.count("control_mode") ? params.at("control_mode") : "position");
  if (!mode) return false;
  hand_mode_ = *mode;
#ifndef GHAND_HAS_ZLG_CAN
  (void)params;
  return false;
#else
  auto it = params.find("device");
  if (it == params.end() || it->second.empty()) {
    return false;
  }
  const std::string device_name = it->second;

  const std::vector<std::string> parts = split(device_name, ':');
  if (parts.size() < 3) {
    return false;
  }

  const int device_type = resolve_vci_device_type(parts[0]);
  const int device_index = parse_int_default(parts[1], 0);
  const int can_index = parse_int_default(parts[2], 0);
  if (device_type == 0) {
    return false;
  }

  impl_->device_type_ = static_cast<uint32_t>(device_type);
  impl_->device_index_ = static_cast<uint32_t>(device_index);
  impl_->can_index_ = static_cast<uint32_t>(can_index);

  const int bitrate =
      parse_int_default(params.count("baud_rate") ? params.at("baud_rate")
                        : params.count("bitrate") ? params.at("bitrate")
                                                  : "",
                        kDefaultBitrate);
  const int dbitrate =
      parse_int_default(params.count("data_rate")   ? params.at("data_rate")
                        : params.count("dbit_rate") ? params.at("dbit_rate")
                                                    : "",
                        kDefaultDataBitrate);
  const uint32_t clock_hz = static_cast<uint32_t>(parse_int_default(
      params.count("can_clk") ? params.at("can_clk") : "", kDefaultCanClock));

  const int device_id_param =
      parse_int_default(params.count("node_id")     ? params.at("node_id")
                        : params.count("device_id") ? params.at("device_id")
                                                    : "",
                        0);
  impl_->device_id_filter_ = static_cast<uint8_t>(device_id_param);
  impl_->dst_id_ = static_cast<uint8_t>(device_id_param != 0 ? device_id_param
                                                             : kDefaultDstId);

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

  if (VCI_OpenDevice(impl_->device_type_, impl_->device_index_, 0) != 1) {
    return false;
  }
  impl_->device_opened_ = true;

  ZCAN_INIT init_config;
  memset(&init_config, 0, sizeof(init_config));
  init_config.clk = clock_hz;
  init_config.mode = 0;

  uint8_t abit_tseg1 = 0, abit_tseg2 = 0, abit_sjw = 0;
  uint16_t abit_brp = 0;
  if (!compute_can_timing(clock_hz, static_cast<uint32_t>(bitrate), &abit_tseg1,
                          &abit_tseg2, &abit_sjw, &abit_brp)) {
    VCI_CloseDevice(impl_->device_type_, impl_->device_index_);
    impl_->device_opened_ = false;
    return false;
  }
  init_config.aset.tseg1 = abit_tseg1;
  init_config.aset.tseg2 = abit_tseg2;
  init_config.aset.sjw = abit_sjw;
  init_config.aset.smp = 0;
  init_config.aset.brp = abit_brp;

  uint8_t dbit_tseg1 = 0, dbit_tseg2 = 0, dbit_sjw = 0;
  uint16_t dbit_brp = 0;
  if (!compute_can_timing(clock_hz, static_cast<uint32_t>(dbitrate),
                          &dbit_tseg1, &dbit_tseg2, &dbit_sjw, &dbit_brp)) {
    VCI_CloseDevice(impl_->device_type_, impl_->device_index_);
    impl_->device_opened_ = false;
    return false;
  }
  init_config.dset.tseg1 = dbit_tseg1;
  init_config.dset.tseg2 = dbit_tseg2;
  init_config.dset.sjw = dbit_sjw;
  init_config.dset.smp = 0;
  init_config.dset.brp = dbit_brp;

  if (VCI_InitCAN(impl_->device_type_, impl_->device_index_, impl_->can_index_,
                  &init_config) != 1) {
    VCI_CloseDevice(impl_->device_type_, impl_->device_index_);
    impl_->device_opened_ = false;
    return false;
  }

  // Enable on-board termination resistor.
  U8 tres = 1;
  VCI_SetReference(impl_->device_type_, impl_->device_index_, impl_->can_index_,
                   CMD_CAN_TRES, &tres);

  if (VCI_StartCAN(impl_->device_type_, impl_->device_index_,
                   impl_->can_index_) != 1) {
    VCI_ResetCAN(impl_->device_type_, impl_->device_index_, impl_->can_index_);
    VCI_CloseDevice(impl_->device_type_, impl_->device_index_);
    impl_->device_opened_ = false;
    return false;
  }

  impl_->connected_.store(true);
  impl_->run_thread_ = true;
  impl_->recv_thread_ = std::thread([this]() { impl_->ReceiveLoop(); });
  RCLCPP_INFO(rclcpp::get_logger("canfd"),
              "initialized: device=%s baud_rate=%u data_rate=%u node_id=%u",
              device_name.c_str(), static_cast<uint32_t>(bitrate),
              static_cast<uint32_t>(dbitrate), impl_->device_id_filter_);
  return true;
#endif
}

void CanfdDriver::close() {
#ifdef GHAND_HAS_ZLG_CAN
  impl_->run_thread_ = false;
  if (impl_->recv_thread_.joinable()) {
    impl_->recv_thread_.join();
  }

  if (impl_->device_opened_) {
    VCI_ResetCAN(impl_->device_type_, impl_->device_index_, impl_->can_index_);
    VCI_CloseDevice(impl_->device_type_, impl_->device_index_);
    impl_->device_opened_ = false;
  }
#endif
  impl_->connected_.store(false);
}

bool CanfdDriver::read(CommunicationData& out_data) {
  std::lock_guard<std::mutex> lock(impl_->data_mutex_);
  out_data = impl_->latest_data_;
  return impl_->connected_.load() && out_data.valid;
}

bool CanfdDriver::write(const std::vector<JointCommand>& commands) {
#ifdef GHAND_HAS_ZLG_CAN
  if (!impl_->connected_.load() || commands.empty()) {
    return true;
  }

  if (!impl_->WriteHoldingRegister(
          0x0260, encode_hand_mode(hand_mode_))) {
    return false;
  }

  for (const auto& block : encode_joint_command_blocks(commands, holding_register_map())) {
    if (!impl_->WriteHoldingRegisters(block.first, registers_to_bytes(block.second))) {
      return false;
    }
  }
  return true;
#else
  (void)commands;
  return true;
#endif
}

#ifdef GHAND_HAS_ZLG_CAN

bool CanfdDriver::Impl::TransmitFrame(uint32_t can_id, const uint8_t* data,
                                      uint8_t len) {
  if (!device_opened_) {
    return false;
  }

  if (len > 64) {
    len = 64;
  }

  ZCAN_FD_MSG tx;
  memset(&tx, 0, sizeof(tx));
  tx.hdr.id = can_id;
  tx.hdr.inf.sef = 1;  // extended frame
  tx.hdr.inf.fmt = 1;  // CANFD
  tx.hdr.inf.brs = 1;  // bit-rate switch
  tx.hdr.inf.sdf = 0;  // data frame
  tx.hdr.len = canfd::nearest_valid_canfd_length(len);
  tx.hdr.chn = static_cast<U8>(can_index_);
  memcpy(tx.dat, data, len);

  std::lock_guard<std::mutex> lock(io_mutex_);
  const U32 sent =
      VCI_TransmitFD(device_type_, device_index_, can_index_, &tx, 1);
  return sent == 1;
}

bool CanfdDriver::Impl::WriteHoldingRegister(uint16_t addr, uint16_t value) {
  const auto payload = canfd::encode_write_single_register(addr, value);
  const uint32_t can_id = canfd::pack_modbus_arbitration_id(
      src_id_, dst_id_, 0, canfd::modbus::kWriteSingleRegister, 1, 1, 0, 0);
  return TransmitFrame(can_id, payload.data(),
                       static_cast<uint8_t>(payload.size()));
}

bool CanfdDriver::Impl::WriteHoldingRegisters(
    uint16_t addr, const std::vector<uint8_t>& data) {
  const auto payload = canfd::encode_write_multiple_registers(addr, data);
  const uint32_t can_id = canfd::pack_modbus_arbitration_id(
      src_id_, dst_id_, 0, canfd::modbus::kWriteMultipleRegisters, 1, 1, 0, 0);
  return TransmitFrame(can_id, payload.data(),
                       static_cast<uint8_t>(payload.size()));
}

void CanfdDriver::Impl::ReceiveLoop() {
  // In the VCI API, CANFD frames are queried by adding 0x80000000 to the
  // channel index.
  const U32 fd_channel = 0x80000000u + can_index_;

  while (run_thread_.load()) {
    if (!device_opened_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    const U32 available =
        VCI_GetReceiveNum(device_type_, device_index_, fd_channel);
    if (available == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    const U32 count = std::min<U32>(available, 64);
    std::vector<ZCAN_FD_MSG> rx_buffer(count);
    const U32 received = VCI_ReceiveFD(device_type_, device_index_, can_index_,
                                       rx_buffer.data(), count, 10);

    for (U32 i = 0; i < received; ++i) {
      const ZCAN_FD_MSG& rx = rx_buffer[i];

      canfd::Frame frame;
      frame.id = rx.hdr.id & 0x1FFFFFFFu;
      frame.len = rx.hdr.len;
      if (frame.len > 64) frame.len = 64;
      memcpy(frame.data, rx.dat, frame.len);
      frame.is_fd = true;
      frame.is_extended = rx.hdr.inf.sef == 1;

      canfd::ArbitrationId arb(frame.id);
      if (arb.frame_type() != canfd::FrameType::ACTIVE_REPORT) {
        continue;
      }

      if (device_id_filter_ != 0 && arb.device_id() != device_id_filter_) {
        continue;
      }

      std::vector<uint8_t> payload;
      if (assembler_.feed(frame, &payload)) {
        ParseReport(payload);
      }
    }
  }
}

void CanfdDriver::Impl::ParseReport(const std::vector<uint8_t>& payload) {
  if (payload.empty()) return;

  const uint8_t func_code = payload[0];
  if (func_code == 0x91) {
    ParseJointReport(payload);
  } else if (func_code == 0x92) {
    ParseTactileReport(payload);
  }
}

void CanfdDriver::Impl::ParseJointReport(const std::vector<uint8_t>& payload) {
  // Payload layout (after function code):
  //   6 bytes per joint: state, error, angle_hi, angle_lo, velocity, current
  std::vector<JointState> all_joints;
  for (size_t i = 1; i + 5 < payload.size(); i += 6) {
    const uint8_t* p = payload.data() + i;
    JointState joint;
    joint.status = p[0];
    joint.error = p[1];
    joint.position = static_cast<int16_t>((p[2] << 8) | p[3]) / 10.0f;
    joint.velocity = static_cast<int8_t>(p[4]);
    joint.current = static_cast<int8_t>(p[5]);
    // Legacy reports do not carry mode/loop feedback. Do not pass their
    // three-word payload to the five-word register feedback decoder.
    all_joints.push_back(joint);
  }

  CommunicationData data;
  data.valid = true;
  data.joints.reserve(valid_joints_.size());
  for (auto id : valid_joints_) {
    const size_t idx = static_cast<size_t>(id);
    if (idx < all_joints.size()) {
      data.joints.push_back(all_joints[idx]);
    } else {
      data.joints.emplace_back();
    }
  }

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_data_.joints = std::move(data.joints);
    latest_data_.valid = true;
  }
}

void CanfdDriver::Impl::ParseTactileReport(
    const std::vector<uint8_t>& payload) {
  // Payload layout (after function code):
  //   sensor_state (1), sensor_error (1),
  //   per region resultant force (6 bytes),
  //   per region distributed forces (region.sensor_count * 3 bytes)
  if (payload.size() < 3) return;

  const size_t header_size = 2;
  const size_t region_count =
      tactile_regions_.empty() ? 5 : tactile_regions_.size();
  const size_t resultant_size = region_count * 6;

  if (payload.size() < 1 + header_size + resultant_size) return;

  std::vector<TactileRegion> regions;
  regions.reserve(region_count);

  // parse_tactile_resultant expects regs[0] to be the tactile state/error
  // register and region i data at regs[1 + i * 3 .. 1 + i * 3 + 2].
  std::vector<uint16_t> regs(1 + region_count * 3, 0);

  for (size_t i = 0; i < region_count; ++i) {
    const size_t r_off = 1 + header_size + i * 6;
    regs[1 + i * 3 + 0] =
        static_cast<uint16_t>((payload[r_off] << 8) | payload[r_off + 1]);
    regs[1 + i * 3 + 1] =
        static_cast<uint16_t>((payload[r_off + 2] << 8) | payload[r_off + 3]);
    regs[1 + i * 3 + 2] =
        static_cast<uint16_t>((payload[r_off + 4] << 8) | payload[r_off + 5]);
  }

  for (size_t i = 0; i < region_count; ++i) {
    TactileRegion tr;
    tr.name = (i < tactile_regions_.size()) ? tactile_regions_[i].first : "";
    tr.resultant_force =
        parse_tactile_resultant(regs.data(), static_cast<int>(i));

    const size_t sensor_count =
        (i < tactile_regions_.size()) ? tactile_regions_[i].second : 0;
    if (sensor_count > 0) {
      const size_t dist_off = 1 + header_size + region_count * 6;
      size_t dist_start = dist_off;
      for (size_t j = 0; j < i; ++j) {
        const size_t count =
            (j < tactile_regions_.size()) ? tactile_regions_[j].second : 0;
        dist_start += count * 3;
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

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_data_.tactile_regions = std::move(regions);
    latest_data_.valid = true;
  }
}

#endif  // GHAND_HAS_ZLG_CAN

}  // namespace ghand_comm
