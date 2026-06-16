#include "communication/rs485_driver.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "communication/communication.hpp"
#include "communication/modbus_codec.hpp"

#ifdef GHAND_HAS_LIBMODBUS
#include <modbus/modbus.h>
#else
struct modbus_t;
inline modbus_t * modbus_new_rtu(const char *, int, char, int, int) { return nullptr; }
inline void modbus_set_response_timeout(modbus_t *, uint32_t, uint32_t) {}
inline void modbus_set_slave(modbus_t *, int) {}
inline int modbus_connect(modbus_t *) { return -1; }
inline void modbus_close(modbus_t *) {}
inline void modbus_free(modbus_t *) {}
inline int modbus_read_registers(modbus_t *, int, int, uint16_t *) { return -1; }
inline int modbus_read_input_registers(modbus_t *, int, int, uint16_t *) { return -1; }
inline int modbus_write_register(modbus_t *, int, uint16_t) { return -1; }
inline int modbus_write_registers(modbus_t *, int, int, const uint16_t *) { return -1; }
#endif

namespace ghand_comm
{

namespace
{

#ifdef GHAND_HAS_LIBMODBUS

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

#endif  // GHAND_HAS_LIBMODBUS

}  // namespace

struct Rs485Driver::Impl
{
  modbus_t * ctx_ = nullptr;
  std::string device_;
  int baud_rate_ = 0;
  uint8_t slave_id_ = 0x31;
  int data_bits_ = 8;
  int stop_bits_ = 1;
  char parity_ = 'N';
  int response_timeout_ms_ = 100;

  std::atomic<bool> connected_{false};
  std::atomic<bool> poll_stop_{false};
  std::thread poll_thread_;

  std::mutex io_mutex_;
  std::mutex data_mutex_;

  std::vector<JointId> valid_joints_;
  std::vector<std::pair<std::string, size_t>> tactile_regions_;
  CommunicationData latest_data_;

  bool ProbeSlave(int sid);
  void CloseContext();
  void PollLoop();

  bool ReadInputRegistersBytes(int addr, int count, std::vector<uint8_t> * bytes);
  bool ReadJoints(std::vector<JointState> * joints);
  bool ReadTactile(std::vector<TactileRegion> * regions);
};

Rs485Driver::Rs485Driver() : impl_(std::make_unique<Impl>()) {}

Rs485Driver::~Rs485Driver()
{
  close();
}

const char * Rs485Driver::get_name() const
{
  return "rs485";
}

bool Rs485Driver::init(const std::unordered_map<std::string, std::string> & params)
{
#ifndef GHAND_HAS_LIBMODBUS
  (void)params;
  return false;
#else
  auto it = params.find("device");
  if (it == params.end() || it->second.empty())
  {
    return false;
  }
  impl_->device_ = it->second;

  impl_->baud_rate_ = parse_int_default(params.count("baud_rate") ? params.at("baud_rate") : "", 0);
  impl_->slave_id_ = static_cast<uint8_t>(
    parse_int_default(params.count("slave_id") ? params.at("slave_id") : "", 0));
  impl_->data_bits_ = parse_int_default(params.count("data_bits") ? params.at("data_bits") : "", 8);
  impl_->stop_bits_ = parse_int_default(params.count("stop_bits") ? params.at("stop_bits") : "", 1);
  impl_->response_timeout_ms_ = parse_int_default(
    params.count("response_timeout_ms") ? params.at("response_timeout_ms") : "", 100);

  if (params.count("parity"))
  {
    const std::string & p = params.at("parity");
    if (!p.empty())
    {
      impl_->parity_ = static_cast<char>(std::toupper(static_cast<unsigned char>(p[0])));
    }
  }

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

  const std::vector<int> candidate_bauds = impl_->baud_rate_ > 0
    ? std::vector<int>{impl_->baud_rate_}
    : std::vector<int>{1000000, 6000000, 115200, 57600, 38400, 19200, 9600};

  for (int baud : candidate_bauds)
  {
    impl_->CloseContext();
    impl_->ctx_ = modbus_new_rtu(
      impl_->device_.c_str(), baud, impl_->parity_, impl_->data_bits_, impl_->stop_bits_);
    if (!impl_->ctx_)
    {
      continue;
    }

    const uint32_t response_us = static_cast<uint32_t>(impl_->response_timeout_ms_ * 1000);
    modbus_set_response_timeout(impl_->ctx_, 0, response_us);

    if (modbus_connect(impl_->ctx_) == -1)
    {
      modbus_free(impl_->ctx_);
      impl_->ctx_ = nullptr;
      continue;
    }

    if (impl_->slave_id_ != 0)
    {
      if (impl_->ProbeSlave(impl_->slave_id_))
      {
        break;
      }
    }
    else
    {
      bool found = false;
      for (int sid : {0x31, 0x32, 0x71, 0x01, 0x02})
      {
        if (impl_->ProbeSlave(sid))
        {
          found = true;
          break;
        }
      }
      if (found)
      {
        break;
      }
    }

    modbus_close(impl_->ctx_);
    modbus_free(impl_->ctx_);
    impl_->ctx_ = nullptr;
  }

  if (!impl_->ctx_ || !impl_->connected_.load())
  {
    impl_->CloseContext();
    return false;
  }

  impl_->poll_stop_ = false;
  impl_->poll_thread_ = std::thread([this]() { impl_->PollLoop(); });
  return true;
#endif
}

void Rs485Driver::close()
{
#ifdef GHAND_HAS_LIBMODBUS
  impl_->poll_stop_ = true;
  if (impl_->poll_thread_.joinable())
  {
    impl_->poll_thread_.join();
  }
  impl_->CloseContext();
#endif
  impl_->connected_ = false;
}

bool Rs485Driver::read(CommunicationData & out_data)
{
  std::lock_guard<std::mutex> lock(impl_->data_mutex_);
  out_data = impl_->latest_data_;
  return impl_->connected_.load() && out_data.valid;
}

bool Rs485Driver::write(const std::vector<JointCommand> & commands)
{
#ifndef GHAND_HAS_LIBMODBUS
  (void)commands;
  return false;
#else
  if (!impl_->connected_.load() || !impl_->ctx_)
  {
    return false;
  }

  std::lock_guard<std::mutex> lock(impl_->io_mutex_);
  modbus_set_slave(impl_->ctx_, impl_->slave_id_);

  if (modbus_write_register(impl_->ctx_, 0x0010, 0x0000) != 1)
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
    const uint16_t data[2] = {regs.first, regs.second};
    if (modbus_write_registers(impl_->ctx_, it->second, 2, data) != 2)
    {
      return false;
    }
  }
  return true;
#endif
}

#ifdef GHAND_HAS_LIBMODBUS

bool Rs485Driver::Impl::ProbeSlave(int sid)
{
  if (!ctx_) return false;

  modbus_set_slave(ctx_, sid);
  uint16_t reg = 0;
  int rc = modbus_read_input_registers(ctx_, 0x1000, 1, &reg);
  if (rc != 1)
  {
    rc = modbus_read_registers(ctx_, 0x0000, 1, &reg);
  }

  if (rc == 1)
  {
    slave_id_ = static_cast<uint8_t>(sid);
    connected_.store(true);
    return true;
  }
  return false;
}

void Rs485Driver::Impl::CloseContext()
{
  if (!ctx_) return;
  modbus_close(ctx_);
  modbus_free(ctx_);
  ctx_ = nullptr;
  connected_.store(false);
}

bool Rs485Driver::Impl::ReadInputRegistersBytes(
  int addr, int count, std::vector<uint8_t> * bytes)
{
  if (!ctx_ || bytes == nullptr) return false;

  std::lock_guard<std::mutex> lock(io_mutex_);
  modbus_set_slave(ctx_, slave_id_);
  std::vector<uint16_t> regs(count);
  if (modbus_read_input_registers(ctx_, addr, count, regs.data()) != count)
  {
    return false;
  }
  *bytes = registers_to_bytes(regs);
  return true;
}

bool Rs485Driver::Impl::ReadJoints(std::vector<JointState> * joints)
{
  if (!joints || valid_joints_.empty()) return false;

  uint8_t max_id = 0;
  for (auto id : valid_joints_)
  {
    max_id = std::max(max_id, static_cast<uint8_t>(id));
  }
  const int count = (max_id + 1) * 3;

  std::vector<uint8_t> bytes;
  if (!ReadInputRegistersBytes(0x1023, count, &bytes))
  {
    return false;
  }

  std::vector<uint16_t> regs;
  regs.reserve(bytes.size() / 2);
  for (size_t i = 0; i + 1 < bytes.size(); i += 2)
  {
    regs.push_back(static_cast<uint16_t>((bytes[i] << 8) | bytes[i + 1]));
  }

  *joints = parse_joints(regs.data(), regs.size(), valid_joints_);
  return !joints->empty();
}

bool Rs485Driver::Impl::ReadTactile(std::vector<TactileRegion> * regions)
{
  if (!regions || tactile_regions_.empty()) return false;

  std::vector<uint8_t> bytes;
  if (!ReadInputRegistersBytes(0x1080, 16, &bytes))
  {
    return false;
  }

  std::vector<uint16_t> regs;
  regs.reserve(bytes.size() / 2);
  for (size_t i = 0; i + 1 < bytes.size(); i += 2)
  {
    regs.push_back(static_cast<uint16_t>((bytes[i] << 8) | bytes[i + 1]));
  }

  const auto tactile_err = parse_tactile_state_error(regs[0]);
  (void)tactile_err;

  regions->clear();
  regions->reserve(tactile_regions_.size());

  int current_addr = 0x1080 + 16;
  for (size_t i = 0; i < tactile_regions_.size(); ++i)
  {
    const auto & region = tactile_regions_[i];
    TactileRegion tr;
    tr.name = region.first;
    tr.resultant_force = parse_tactile_resultant(regs.data(), static_cast<int>(i));

    const int dist_regs = static_cast<int>((region.second * 3 + 1) / 2);
    std::vector<uint8_t> dist_bytes;
    if (ReadInputRegistersBytes(current_addr, dist_regs, &dist_bytes))
    {
      tr.distributed_forces = parse_tactile_distributed(
        dist_bytes.data(), dist_bytes.size(), static_cast<int>(region.second));
    }
    current_addr += dist_regs;
    regions->push_back(std::move(tr));
  }
  return true;
}

void Rs485Driver::Impl::PollLoop()
{
  auto last_tactile_time = std::chrono::steady_clock::time_point{};

  while (!poll_stop_.load())
  {
    if (!connected_.load() || !ctx_)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    CommunicationData data;
    data.valid = true;

    if (ReadJoints(&data.joints))
    {
      data.valid = true;
    }
    else
    {
      data.valid = false;
    }

    auto now = std::chrono::steady_clock::now();
    if (!tactile_regions_.empty() &&
        (last_tactile_time.time_since_epoch().count() == 0 ||
         now - last_tactile_time >= std::chrono::milliseconds(100)))
    {
      std::vector<TactileRegion> tactile;
      if (ReadTactile(&tactile))
      {
        data.tactile_regions = std::move(tactile);
        last_tactile_time = now;
      }
    }

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_data_ = std::move(data);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

#endif  // GHAND_HAS_LIBMODBUS

}  // namespace ghand_comm
