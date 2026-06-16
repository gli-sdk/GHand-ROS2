#include "communication/ethercat_driver.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

#include "communication/modbus_codec.hpp"

#ifdef GHAND_HAS_SOEM
#include <soem/soem.h>
#endif

namespace ghand_comm
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr int kEcTimeoutMon = 500;

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

}  // namespace

struct EthercatDriver::Impl
{
#ifdef GHAND_HAS_SOEM
  ecx_contextt ctx{};
  uint8_t io_map[4096] = {0};
  OSAL_THREAD_HANDLE threadrt{};
  OSAL_THREAD_HANDLE thread1{};
#endif

  int expected_wkc = 0;
  int wkc = 0;
  int mapping_done = 0;
  std::atomic<int> do_run{0};
  int in_op = 0;
  bool is_connected = false;
  int wkc_check = 0;
  uint8_t rxpdo_buffer[80] = {0};

  std::mutex rt_mutex;
  std::mutex data_mutex;

  std::string ifname;
  std::vector<JointId> valid_joints;
  std::vector<std::pair<std::string, size_t>> tactile_regions;
  CommunicationData latest_data;

  std::atomic<bool> threads_started{false};
  std::thread rt_thread;
  std::thread check_thread;

  void ecat_thread();
  void ecat_check();
  void parse_and_notify(const uint8_t * data, size_t size);
  void parse_hand_and_joints(
    const uint8_t * data, size_t * offset, CommunicationData * out_data) const;
  void parse_tactile_data(
    const uint8_t * data, size_t size, size_t * offset,
    CommunicationData * out_data) const;
};

EthercatDriver::EthercatDriver() : impl_(std::make_unique<Impl>()) {}

EthercatDriver::~EthercatDriver()
{
  close();
}

const char * EthercatDriver::get_name() const
{
  return "ethercat";
}

bool EthercatDriver::init(const std::unordered_map<std::string, std::string> & params)
{
#ifndef GHAND_HAS_SOEM
  (void)params;
  return false;
#else
  auto it = params.find("ifname");
  if (it == params.end() || it->second.empty())
  {
    return false;
  }
  impl_->ifname = it->second;

  auto jt = params.find("joint_names");
  if (jt != params.end())
  {
    for (const auto & name : split(jt->second, ','))
    {
      const auto id = joint_name_to_id(name);
      if (id.has_value())
      {
        impl_->valid_joints.push_back(id.value());
      }
    }
  }
  // If no mapping provided, assume full SDK ordering.
  if (impl_->valid_joints.empty())
  {
    for (uint8_t i = 0; i < static_cast<uint8_t>(JointId::NUM_JOINTS); ++i)
    {
      impl_->valid_joints.push_back(static_cast<JointId>(i));
    }
  }

  auto tt = params.find("tactile_region_counts");
  if (tt != params.end())
  {
    const auto counts = split(tt->second, ',');
    const std::vector<std::string> names = {
      "THUMB", "FF", "MF", "RF", "LF"};
    for (size_t i = 0; i < counts.size() && i < names.size(); ++i)
    {
      impl_->tactile_regions.emplace_back(names[i], std::stoul(counts[i]));
    }
  }

  std::memset(impl_->rxpdo_buffer, 0, sizeof(impl_->rxpdo_buffer));
  impl_->rxpdo_buffer[1] = 0x01;

  std::lock_guard<std::mutex> lock(impl_->rt_mutex);

  std::memset(&impl_->ctx, 0, sizeof(ecx_contextt));
  std::memset(impl_->io_map, 0, sizeof(impl_->io_map));

  if (ecx_init(&impl_->ctx, impl_->ifname.c_str()) <= 0)
  {
    return false;
  }

  if (ecx_config_init(&impl_->ctx) <= 0 || impl_->ctx.slavecount <= 0)
  {
    return false;
  }

  if (ecx_config_map_group(&impl_->ctx, impl_->io_map, 0) <= 0)
  {
    return false;
  }

  impl_->mapping_done = 1;
  impl_->do_run = 1;
  ec_groupt * group = &impl_->ctx.grouplist[0];
  impl_->expected_wkc = (group->outputsWKC * 2) + group->inputsWKC;

  ecx_configdc(&impl_->ctx);

  for (int si = 1; si <= impl_->ctx.slavecount; ++si)
  {
    if (impl_->ctx.slavelist[si].CoEdetails > 0)
    {
      ecx_slavembxcyclic(&impl_->ctx, si);
    }
  }

  if (ecx_statecheck(
        &impl_->ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE) !=
      EC_STATE_SAFE_OP)
  {
    return false;
  }

  impl_->ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
  ecx_writestate(&impl_->ctx, 0);

  impl_->threads_started = true;
  impl_->rt_thread = std::thread([this]() { impl_->ecat_thread(); });
  impl_->check_thread = std::thread([this]() { impl_->ecat_check(); });

  if (ecx_statecheck(
        &impl_->ctx, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE) !=
      EC_STATE_OPERATIONAL)
  {
    close();
    return false;
  }

  impl_->in_op = 1;
  impl_->is_connected = true;
  return true;
#endif
}

void EthercatDriver::close()
{
#ifdef GHAND_HAS_SOEM
  impl_->threads_started = false;
  impl_->do_run = 0;
  impl_->in_op = 0;

  if (impl_->rt_thread.joinable()) impl_->rt_thread.join();
  if (impl_->check_thread.joinable()) impl_->check_thread.join();

  {
    std::lock_guard<std::mutex> lock(impl_->rt_mutex);
    if (impl_->ctx.slavecount > 0)
    {
      impl_->ctx.slavelist[0].state = EC_STATE_INIT;
      ecx_writestate(&impl_->ctx, 0);
    }
  }
#endif
  impl_->is_connected = false;
}

bool EthercatDriver::read(CommunicationData & out_data)
{
  std::lock_guard<std::mutex> lock(impl_->data_mutex);
  out_data = impl_->latest_data;
  return impl_->is_connected && out_data.valid;
}

bool EthercatDriver::write(const std::vector<JointCommand> & commands)
{
#ifdef GHAND_HAS_SOEM
  if (!impl_->is_connected) return false;

  std::vector<uint8_t> buffer(
    impl_->valid_joints.size() * 6 + 2, 0);
  size_t offset = 0;
  buffer[offset++] = 0x00;  // position control mode
  buffer[offset++] = 0x00;  // stop flag

  for (const auto & joint_id : impl_->valid_joints)
  {
    float angle = 0.0f;
    uint8_t velocity = 0;
    uint8_t torque = 0;

    for (const auto & cmd : commands)
    {
      if (cmd.id == joint_id)
      {
        angle = cmd.position * static_cast<float>(kPi) / 180.0f;
        velocity = static_cast<uint8_t>(cmd.velocity);
        torque = static_cast<uint8_t>(cmd.torque);
        break;
      }
    }

    std::memcpy(buffer.data() + offset, &angle, sizeof(angle));
    offset += sizeof(angle);
    buffer[offset++] = velocity;
    buffer[offset++] = torque;
  }

  const size_t copy_size =
    std::min(buffer.size(), sizeof(impl_->rxpdo_buffer));
  {
    std::lock_guard<std::mutex> lock(impl_->rt_mutex);
    std::memcpy(impl_->rxpdo_buffer, buffer.data(), copy_size);
  }
  return true;
#else
  (void)commands;
  return false;
#endif
}

#ifdef GHAND_HAS_SOEM

void EthercatDriver::Impl::ecat_thread()
{
  ec_timet ts;
  int ht;

  while (!mapping_done)
  {
    osal_usleep(500);
  }

  osal_get_monotonic_time(&ts);
  ht = (ts.tv_nsec / 1000000) + 1;
  ts.tv_nsec = ht * 1000000;

  {
    std::lock_guard<std::mutex> lock(rt_mutex);
    if (ctx.slavecount > 0)
    {
      std::memcpy(
        ctx.slavelist[1].outputs, rxpdo_buffer, sizeof(rxpdo_buffer));
    }
    ecx_send_processdata(&ctx);
  }

  while (threads_started)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    if (do_run > 0)
    {
      std::vector<uint8_t> pdo_copy;

      {
        std::lock_guard<std::mutex> lock(rt_mutex);
        wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);

        if (wkc >= expected_wkc && in_op && ctx.slavecount > 0)
        {
          uint8_t * inputs = ctx.slavelist[1].inputs;
          size_t pdo_size = ctx.slavelist[1].Ibytes;
          if (pdo_size > 0)
          {
            pdo_copy.assign(inputs, inputs + pdo_size);
          }
        }
      }

      if (!pdo_copy.empty())
      {
        parse_and_notify(pdo_copy.data(), pdo_copy.size());
      }

      if (wkc != expected_wkc)
      {
        ++wkc_check;
      }
      else
      {
        wkc_check = 0;
      }

      {
        std::lock_guard<std::mutex> lock(rt_mutex);
        if (ctx.slavecount > 0)
        {
          std::memcpy(
            ctx.slavelist[1].outputs, rxpdo_buffer, sizeof(rxpdo_buffer));
        }
        ecx_send_processdata(&ctx);
      }
    }
  }
}

void EthercatDriver::Impl::ecat_check()
{
  while (threads_started)
  {
    osal_usleep(50000);

    if (in_op &&
        (wkc_check > 2 || ctx.grouplist[0].docheckstate))
    {
      std::unique_lock<std::mutex> lock(rt_mutex, std::try_to_lock);
      if (!lock.owns_lock()) continue;

      ctx.grouplist[0].docheckstate = FALSE;
      ecx_readstate(&ctx);
      for (int slaveix = 1; slaveix <= ctx.slavecount; ++slaveix)
      {
        ec_slavet * slave = &ctx.slavelist[slaveix];
        if ((slave->group == 0) &&
            (slave->state != EC_STATE_OPERATIONAL))
        {
          ctx.grouplist[0].docheckstate = TRUE;
          if (slave->state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
          {
            slave->state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
            ecx_writestate(&ctx, slaveix);
          }
          else if (slave->state == EC_STATE_SAFE_OP)
          {
            slave->state = EC_STATE_OPERATIONAL;
            if (slave->mbxhandlerstate == ECT_MBXH_LOST)
            {
              slave->mbxhandlerstate = ECT_MBXH_CYCLIC;
            }
            ecx_writestate(&ctx, slaveix);
          }
          else if (slave->state > EC_STATE_NONE)
          {
            if (ecx_reconfig_slave(&ctx, slaveix, kEcTimeoutMon) >=
                EC_STATE_PRE_OP)
            {
              slave->islost = FALSE;
            }
          }
          else if (!slave->islost)
          {
            ecx_statecheck(&ctx, slaveix, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (slave->state == EC_STATE_NONE)
            {
              slave->islost = TRUE;
              threads_started = false;
              is_connected = false;
              slave->mbxhandlerstate = ECT_MBXH_LOST;
              if (slave->Ibytes)
              {
                std::memset(slave->inputs, 0x00, slave->Ibytes);
              }
            }
          }
        }
        if (slave->islost)
        {
          if (slave->state <= EC_STATE_INIT)
          {
            if (ecx_recover_slave(&ctx, slaveix, kEcTimeoutMon))
            {
              slave->islost = FALSE;
            }
          }
          else
          {
            slave->islost = FALSE;
          }
        }
      }
      wkc_check = 0;
    }
  }
}

namespace
{

Force parse_resultant_force(const uint8_t * data)
{
  Force resultant;
  if (data != nullptr)
  {
    const int16_t raw_x = static_cast<int16_t>(data[0] | (data[1] << 8));
    resultant.x = static_cast<float>(static_cast<int8_t>(raw_x & 0xFF)) * 0.1f;

    const int16_t raw_y = static_cast<int16_t>(data[2] | (data[3] << 8));
    resultant.y = static_cast<float>(static_cast<int8_t>(raw_y & 0xFF)) * 0.1f;

    const uint16_t raw_z = static_cast<uint16_t>(data[4] | (data[5] << 8));
    resultant.z = static_cast<float>(static_cast<uint8_t>(raw_z & 0xFF)) * 0.1f;
  }
  return resultant;
}

std::vector<Force> parse_sample_forces(const uint8_t * data, int sensor_count)
{
  std::vector<Force> forces;
  forces.reserve(sensor_count);
  if (data != nullptr && sensor_count > 0)
  {
    for (int i = 0; i < sensor_count; ++i)
    {
      Force f;
      f.x = static_cast<float>(static_cast<int8_t>(data[i * 3])) * 0.1f;
      f.y = static_cast<float>(static_cast<int8_t>(data[i * 3 + 1])) * 0.1f;
      f.z = static_cast<float>(static_cast<uint8_t>(data[i * 3 + 2])) * 0.1f;
      forces.push_back(f);
    }
  }
  return forces;
}

}  // namespace

void EthercatDriver::Impl::parse_hand_and_joints(
  const uint8_t * data, size_t * offset, CommunicationData * out_data) const
{
  uint8_t hand_state = 0;
  uint8_t hand_error = 0;
  int16_t temperature = 0;

  std::memcpy(&hand_state, data + *offset, sizeof(hand_state));
  *offset += sizeof(hand_state);
  std::memcpy(&hand_error, data + *offset, sizeof(hand_error));
  *offset += sizeof(hand_error);
  std::memcpy(&temperature, data + *offset, sizeof(temperature));
  *offset += sizeof(temperature);

  (void)hand_state;
  (void)hand_error;
  (void)temperature;

  out_data->joints.reserve(valid_joints.size());
  for (const auto joint_id : valid_joints)
  {
    uint8_t joint_state = 0;
    uint8_t joint_error = 0;
    float joint_angle = 0.0f;
    uint8_t joint_velocity = 0;
    uint8_t joint_torque = 0;

    std::memcpy(&joint_state, data + *offset, sizeof(joint_state));
    *offset += sizeof(joint_state);
    std::memcpy(&joint_error, data + *offset, sizeof(joint_error));
    *offset += sizeof(joint_error);
    std::memcpy(&joint_angle, data + *offset, sizeof(joint_angle));
    *offset += sizeof(joint_angle);
    std::memcpy(&joint_velocity, data + *offset, sizeof(joint_velocity));
    *offset += sizeof(joint_velocity);
    std::memcpy(&joint_torque, data + *offset, sizeof(joint_torque));
    *offset += sizeof(joint_torque);

    JointState js;
    js.status = joint_state;
    js.error = joint_error;
    js.position = joint_angle * (180.0f / static_cast<float>(kPi));
    js.velocity = static_cast<float>(static_cast<int8_t>(joint_velocity));
    (void)joint_torque;
    out_data->joints.push_back(js);
  }
}

void EthercatDriver::Impl::parse_tactile_data(
  const uint8_t * data, size_t size, size_t * offset,
  CommunicationData * out_data) const
{
  if (*offset + 2 <= size)
  {
    const uint8_t sensor_state = data[*offset];
    const uint8_t sensor_error = data[*offset + 1];
    *offset += 2;
    (void)sensor_state;
    (void)sensor_error;
  }

  out_data->tactile_regions.reserve(tactile_regions.size());
  for (const auto & region : tactile_regions)
  {
    TactileRegion tr;
    tr.name = region.first;

    if (*offset + 6 <= size)
    {
      tr.resultant_force = parse_resultant_force(data + *offset);
      *offset += 6;
    }

    const int sample_size = static_cast<int>(region.second) * 3;
    if (region.second > 0 && *offset + sample_size <= size)
    {
      tr.distributed_forces =
        parse_sample_forces(data + *offset, static_cast<int>(region.second));
      *offset += sample_size;
    }

    out_data->tactile_regions.push_back(std::move(tr));
  }
}

void EthercatDriver::Impl::parse_and_notify(
  const uint8_t * data, size_t size)
{
  if (data == nullptr || size == 0) return;

  CommunicationData parsed;
  parsed.valid = true;
  size_t offset = 0;
  parse_hand_and_joints(data, &offset, &parsed);

  if (!tactile_regions.empty() && offset < size)
  {
    parse_tactile_data(data, size, &offset, &parsed);
  }

  {
    std::lock_guard<std::mutex> lock(data_mutex);
    latest_data = std::move(parsed);
  }
}

#endif  // GHAND_HAS_SOEM

}  // namespace ghand_comm
