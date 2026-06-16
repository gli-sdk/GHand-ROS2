#include "communication/canfd_driver.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "communication/canfd_protocol.hpp"
#include "communication/communication.hpp"
#include "communication/modbus_codec.hpp"

#ifdef GHAND_HAS_ZLG_CAN
#ifdef _WIN32
#include <windows.h>
#endif
#include <zlgcan.h>
#endif

namespace ghand_comm
{

namespace
{

std::vector<std::string> split(const std::string & text, char delimiter)
{
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream stream(text);
  while (std::getline(stream, token, delimiter))
  {
    if (!token.empty()) tokens.push_back(token);
  }
  return tokens;
}

int parse_int_default(const std::string & text, int default_value)
{
  try
  {
    return std::stoi(text);
  }
  catch (...)
  {
    return default_value;
  }
}

#ifdef GHAND_HAS_ZLG_CAN

int resolve_zlg_device_type(const std::string & name)
{
  if (name.empty()) return 0;

  if (std::isdigit(static_cast<unsigned char>(name[0])))
  {
    return parse_int_default(name, 0);
  }

  static const std::map<std::string, int> kNameToType = {
    {"ZCAN_USBCANFD_100U", ZCAN_USBCANFD_100U},
    {"ZCAN_USBCANFD_200U", ZCAN_USBCANFD_200U},
    {"ZCAN_USBCANFD_400U", ZCAN_USBCANFD_400U},
    {"ZCAN_USBCANFD_MINI", ZCAN_USBCANFD_MINI},
    {"ZCAN_USBCANFD_800U", ZCAN_USBCANFD_800U},
    {"ZCAN_PCIE_CANFD_100U", ZCAN_PCIE_CANFD_100U},
    {"ZCAN_PCIE_CANFD_200U", ZCAN_PCIE_CANFD_200U},
    {"ZCAN_PCIE_CANFD_400U", ZCAN_PCIE_CANFD_400U},
    {"ZCAN_PCIE_CANFD_100U_EX", ZCAN_PCIE_CANFD_100U_EX},
    {"ZCAN_PCIE_CANFD_400U_EX", ZCAN_PCIE_CANFD_400U_EX},
    {"ZCAN_PCIE_CANFD_200U_MINI", ZCAN_PCIE_CANFD_200U_MINI},
    {"ZCAN_PCIE_CANFD_200U_M2", ZCAN_PCIE_CANFD_200U_M2},
    {"ZCAN_CANFDCOM_100IE", ZCAN_CANFDCOM_100IE},
  };

  auto it = kNameToType.find(name);
  return (it != kNameToType.end()) ? it->second : 0;
}

#endif  // GHAND_HAS_ZLG_CAN

}  // namespace

struct CanfdDriver::Impl
{
#ifdef GHAND_HAS_ZLG_CAN
  long device_handle_ = INVALID_DEVICE_HANDLE;
  Handle_chl * channel_handle_ = nullptr;

  bool WriteHoldingRegister(uint16_t addr, uint16_t value);
  bool WriteHoldingRegisters(uint16_t addr, const std::vector<uint8_t> & data);
  bool TransmitFrame(uint32_t can_id, const uint8_t * data, uint8_t len);
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
  uint8_t src_id_ = 0x0A;
  uint8_t dst_id_ = 0x31;

  canfd::PacketAssembler assembler_;

  void ReceiveLoop();
  void ParseReport(const std::vector<uint8_t> & payload);
  void ParseJointReport(const std::vector<uint8_t> & payload);
  void ParseTactileReport(const std::vector<uint8_t> & payload);
};

CanfdDriver::CanfdDriver() : impl_(std::make_unique<Impl>()) {}

CanfdDriver::~CanfdDriver()
{
  close();
}

const char * CanfdDriver::get_name() const
{
  return "canfd";
}

bool CanfdDriver::init(const std::unordered_map<std::string, std::string> & params)
{
#ifndef GHAND_HAS_ZLG_CAN
  (void)params;
  return false;
#else
  auto it = params.find("device");
  if (it == params.end() || it->second.empty())
  {
    return false;
  }
  const std::string device_name = it->second;

  const std::vector<std::string> parts = split(device_name, ':');
  if (parts.size() < 3)
  {
    return false;
  }

  const int device_type = resolve_zlg_device_type(parts[0]);
  const int device_index = parse_int_default(parts[1], 0);
  const int can_index = parse_int_default(parts[2], 0);
  if (device_type == 0)
  {
    return false;
  }

  const int bitrate = parse_int_default(
    params.count("bitrate") ? params.at("bitrate") : "", 1000000);
  const int dbitrate = parse_int_default(
    params.count("dbit_rate") ? params.at("dbit_rate") : "", 5000000);
  const int abit_sample_point = parse_int_default(
    params.count("abit_sample_point") ? params.at("abit_sample_point") : "", 75);
  const int dbit_sample_point = parse_int_default(
    params.count("dbit_sample_point") ? params.at("dbit_sample_point") : "", 80);

  const int device_id_param = parse_int_default(
    params.count("device_id") ? params.at("device_id") : "", 0);
  impl_->device_id_filter_ = static_cast<uint8_t>(device_id_param);
  impl_->dst_id_ = static_cast<uint8_t>(device_id_param != 0 ? device_id_param : 0x31);

  auto jt = params.find("joint_names");
  if (jt != params.end())
  {
    for (const auto & name : split(jt->second, ','))
    {
      const auto id = joint_name_to_id(name);
      if (id.has_value())
      {
        impl_->valid_joints_.push_back(id.value());
      }
    }
  }
  if (impl_->valid_joints_.empty())
  {
    for (uint8_t i = 0; i < static_cast<uint8_t>(JointId::NUM_JOINTS); ++i)
    {
      impl_->valid_joints_.push_back(static_cast<JointId>(i));
    }
  }

  auto tt = params.find("tactile_region_counts");
  if (tt != params.end())
  {
    const auto counts = split(tt->second, ',');
    const std::vector<std::string> names = {"THUMB", "FF", "MF", "RF", "LF"};
    for (size_t i = 0; i < counts.size() && i < names.size(); ++i)
    {
      impl_->tactile_regions_.emplace_back(names[i], std::stoul(counts[i]));
    }
  }

  impl_->device_handle_ = ZCAN_OpenDevice(device_type, device_index, 0);
  if (impl_->device_handle_ == INVALID_DEVICE_HANDLE)
  {
    return false;
  }

  IProperty * prop = GetIProperty(impl_->device_handle_);
  if (prop != nullptr)
  {
    const std::string ch_str = std::to_string(can_index);
    prop->SetValue((ch_str + "/canfd_abit_baud_rate").c_str(), std::to_string(bitrate).c_str());
    prop->SetValue((ch_str + "/canfd_dbit_baud_rate").c_str(), std::to_string(dbitrate).c_str());
    prop->SetValue(
      (ch_str + "/canfd_abit_sample_point").c_str(),
      std::to_string(abit_sample_point).c_str());
    prop->SetValue(
      (ch_str + "/canfd_dbit_sample_point").c_str(),
      std::to_string(dbit_sample_point).c_str());

    if (prop->SetValue((ch_str + "/initenal_resistance").c_str(), "1") != 1)
    {
      prop->SetValue((ch_str + "/internal_resistance").c_str(), "1");
    }
  }

  ZCAN_CHANNEL_INIT_CONFIG config;
  memset(&config, 0, sizeof(config));
  config.can_type = TYPE_CANFD;
  config.canfd.abit_timing = 0;
  config.canfd.dbit_timing = 0;
  config.canfd.filter = 0;
  config.canfd.mode = 0;

  impl_->channel_handle_ = ZCAN_InitCAN(
    impl_->device_handle_, static_cast<UINT>(can_index), config);
  if (impl_->channel_handle_ == nullptr)
  {
    ZCAN_CloseDevice(impl_->device_handle_);
    impl_->device_handle_ = INVALID_DEVICE_HANDLE;
    return false;
  }

  if (ZCAN_StartCAN(*impl_->channel_handle_) != STATUS_OK)
  {
    ZCAN_ResetCAN(*impl_->channel_handle_);
    ZCAN_CloseDevice(impl_->device_handle_);
    impl_->channel_handle_ = nullptr;
    impl_->device_handle_ = INVALID_DEVICE_HANDLE;
    return false;
  }

  impl_->connected_.store(true);
  impl_->run_thread_ = true;
  impl_->recv_thread_ = std::thread([this]() { impl_->ReceiveLoop(); });
  return true;
#endif
}

void CanfdDriver::close()
{
#ifdef GHAND_HAS_ZLG_CAN
  impl_->run_thread_ = false;
  if (impl_->recv_thread_.joinable())
  {
    impl_->recv_thread_.join();
  }

  if (impl_->channel_handle_ != nullptr)
  {
    ZCAN_ResetCAN(*impl_->channel_handle_);
    impl_->channel_handle_ = nullptr;
  }
  if (impl_->device_handle_ != INVALID_DEVICE_HANDLE)
  {
    ZCAN_CloseDevice(impl_->device_handle_);
    impl_->device_handle_ = INVALID_DEVICE_HANDLE;
  }
#endif
  impl_->connected_.store(false);
}

bool CanfdDriver::read(CommunicationData & out_data)
{
  std::lock_guard<std::mutex> lock(impl_->data_mutex_);
  out_data = impl_->latest_data_;
  return impl_->connected_.load() && out_data.valid;
}

bool CanfdDriver::write(const std::vector<JointCommand> & commands)
{
#ifdef GHAND_HAS_ZLG_CAN
  if (!impl_->connected_.load() || commands.empty())
  {
    return true;
  }

  if (!impl_->WriteHoldingRegister(0x0010, 0x0000))
  {
    return false;
  }

  for (const auto & cmd : commands)
  {
    const auto & reg_map = holding_register_map();
    auto it = reg_map.find(cmd.id);
    if (it == reg_map.end())
    {
      continue;
    }

    const auto regs = encode_joint_command(cmd);
    const std::vector<uint8_t> data = {
      static_cast<uint8_t>((regs.first >> 8) & 0xFF),
      static_cast<uint8_t>(regs.first & 0xFF),
      static_cast<uint8_t>((regs.second >> 8) & 0xFF),
      static_cast<uint8_t>(regs.second & 0xFF)};

    if (!impl_->WriteHoldingRegisters(it->second, data))
    {
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

bool CanfdDriver::Impl::TransmitFrame(uint32_t can_id, const uint8_t * data, uint8_t len)
{
  if (channel_handle_ == nullptr)
  {
    return false;
  }

  ZCAN_TransmitFD_Data tx;
  memset(&tx, 0, sizeof(tx));
  tx.frame.can_id = MAKE_CAN_ID(can_id, 1, 0, 0);
  tx.frame.len = canfd::nearest_valid_canfd_length(len);
  tx.frame.flags = CANFD_BRS;
  if (len > 64)
  {
    len = 64;
  }
  memcpy(tx.frame.data, data, len);
  tx.transmit_type = 0;

  std::lock_guard<std::mutex> lock(io_mutex_);
  const UINT sent = ZCAN_TransmitFD(*channel_handle_, &tx, 1);
  return sent == 1;
}

bool CanfdDriver::Impl::WriteHoldingRegister(uint16_t addr, uint16_t value)
{
  const auto payload = canfd::encode_write_single_register(addr, value);
  const uint32_t can_id = canfd::pack_modbus_arbitration_id(
    src_id_, dst_id_, 0, canfd::modbus::kWriteSingleRegister, 1, 1, 0, 0);
  return TransmitFrame(can_id, payload.data(), static_cast<uint8_t>(payload.size()));
}

bool CanfdDriver::Impl::WriteHoldingRegisters(uint16_t addr, const std::vector<uint8_t> & data)
{
  const auto payload = canfd::encode_write_multiple_registers(addr, data);
  const uint32_t can_id = canfd::pack_modbus_arbitration_id(
    src_id_, dst_id_, 0, canfd::modbus::kWriteMultipleRegisters, 1, 1, 0, 0);
  return TransmitFrame(can_id, payload.data(), static_cast<uint8_t>(payload.size()));
}

void CanfdDriver::Impl::ReceiveLoop()
{
  while (run_thread_.load())
  {
    if (!channel_handle_)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    ZCAN_ReceiveFD_Data rx;
    memset(&rx, 0, sizeof(rx));

    const UINT received = ZCAN_ReceiveFD(*channel_handle_, &rx, 1, 10);
    if (received == 0)
    {
      continue;
    }

    canfd::Frame frame;
    frame.id = GET_ID(rx.frame.can_id);
    frame.len = rx.frame.len;
    if (frame.len > 64) frame.len = 64;
    memcpy(frame.data, rx.frame.data, frame.len);
    frame.is_fd = true;
    frame.is_extended = IS_EFF(rx.frame.can_id);

    canfd::ArbitrationId arb(frame.id);
    if (arb.frame_type() != canfd::FrameType::ACTIVE_REPORT)
    {
      continue;
    }

    if (device_id_filter_ != 0 && arb.device_id() != device_id_filter_)
    {
      continue;
    }

    std::vector<uint8_t> payload;
    if (assembler_.feed(frame, &payload))
    {
      ParseReport(payload);
    }
  }
}

void CanfdDriver::Impl::ParseReport(const std::vector<uint8_t> & payload)
{
  if (payload.empty()) return;

  const uint8_t func_code = payload[0];
  if (func_code == 0x91)
  {
    ParseJointReport(payload);
  }
  else if (func_code == 0x92)
  {
    ParseTactileReport(payload);
  }
}

void CanfdDriver::Impl::ParseJointReport(const std::vector<uint8_t> & payload)
{
  // Payload layout (after function code):
  //   6 bytes per joint: state, error, angle_hi, angle_lo, velocity, torque
  std::vector<JointState> all_joints;
  for (size_t i = 1; i + 5 < payload.size(); i += 6)
  {
    const uint8_t * p = payload.data() + i;
    uint16_t raw[3] = {
      static_cast<uint16_t>((p[0] << 8) | p[1]),
      static_cast<uint16_t>((p[2] << 8) | p[3]),
      static_cast<uint16_t>((p[4] << 8) | p[5]),
    };
    all_joints.push_back(parse_joint_data(raw, static_cast<JointId>(all_joints.size())));
  }

  CommunicationData data;
  data.valid = true;
  data.joints.reserve(valid_joints_.size());
  for (auto id : valid_joints_)
  {
    const size_t idx = static_cast<size_t>(id);
    if (idx < all_joints.size())
    {
      data.joints.push_back(all_joints[idx]);
    }
    else
    {
      data.joints.emplace_back();
    }
  }

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_data_.joints = std::move(data.joints);
    latest_data_.valid = true;
  }
}

void CanfdDriver::Impl::ParseTactileReport(const std::vector<uint8_t> & payload)
{
  // Payload layout (after function code):
  //   sensor_state (1), sensor_error (1),
  //   per region resultant force (6 bytes),
  //   per region distributed forces (region.sensor_count * 3 bytes)
  if (payload.size() < 3) return;

  const size_t header_size = 2;
  const size_t region_count = tactile_regions_.empty() ? 5 : tactile_regions_.size();
  const size_t resultant_size = region_count * 6;

  if (payload.size() < 1 + header_size + resultant_size) return;

  std::vector<TactileRegion> regions;
  regions.reserve(region_count);

  // parse_tactile_resultant expects regs[0] to be the tactile state/error
  // register and region i data at regs[1 + i * 3 .. 1 + i * 3 + 2].
  std::vector<uint16_t> regs(1 + region_count * 3, 0);

  for (size_t i = 0; i < region_count; ++i)
  {
    const size_t r_off = 1 + header_size + i * 6;
    regs[1 + i * 3 + 0] = static_cast<uint16_t>((payload[r_off] << 8) | payload[r_off + 1]);
    regs[1 + i * 3 + 1] = static_cast<uint16_t>((payload[r_off + 2] << 8) | payload[r_off + 3]);
    regs[1 + i * 3 + 2] = static_cast<uint16_t>((payload[r_off + 4] << 8) | payload[r_off + 5]);
  }

  for (size_t i = 0; i < region_count; ++i)
  {
    TactileRegion tr;
    tr.name = (i < tactile_regions_.size()) ? tactile_regions_[i].first : "";
    tr.resultant_force = parse_tactile_resultant(regs.data(), static_cast<int>(i));

    const size_t sensor_count = (i < tactile_regions_.size()) ? tactile_regions_[i].second : 0;
    if (sensor_count > 0)
    {
      const size_t dist_off = 1 + header_size + region_count * 6;
      size_t dist_start = dist_off;
      for (size_t j = 0; j < i; ++j)
      {
        const size_t count = (j < tactile_regions_.size()) ? tactile_regions_[j].second : 0;
        dist_start += count * 3;
      }
      const size_t dist_len = sensor_count * 3;
      if (dist_start + dist_len <= payload.size())
      {
        tr.distributed_forces = parse_tactile_distributed(
          payload.data() + dist_start, dist_len, static_cast<int>(sensor_count));
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
