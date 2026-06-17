#include "communication/ethercat_driver.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>   // 新增
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

// EtherCAT PDO uses 13 active joints in canonical ros2_control order (all joints
// except DIP). These map to the 18-joint canonical order, with DIP joints
// (0, 5, 9, 12, 15) omitted from the PDO.
const std::vector<ghand_comm::JointId> kEthercatActiveJoints = {
  // Slave PDO order is tip-to-base within each finger (DIP -> PIP -> MCP -> ...).
  ghand_comm::JointId::THUMB_PIP,       // thumb_mcp,     ros2_control index 1
  ghand_comm::JointId::THUMB_MCP,       // thumb_tmc_fe,  ros2_control index 2
  ghand_comm::JointId::THUMB_SWING,     // thumb_tmc_aa,  ros2_control index 3
  ghand_comm::JointId::THUMB_ROTATION,  // thumb_tmc_ps,  ros2_control index 4
  ghand_comm::JointId::FF_PIP,          // index_pip,     ros2_control index 6
  ghand_comm::JointId::FF_MCP,          // index_mcp,     ros2_control index 7
  ghand_comm::JointId::FF_SWING,        // index_mcp_aa,  ros2_control index 8
  ghand_comm::JointId::MF_PIP,          // middle_pip,    ros2_control index 10
  ghand_comm::JointId::MF_MCP,          // middle_mcp,    ros2_control index 11
  ghand_comm::JointId::RF_PIP,          // ring_pip,      ros2_control index 13
  ghand_comm::JointId::RF_MCP,          // ring_mcp,      ros2_control index 14
  ghand_comm::JointId::LF_PIP,          // little_pip,    ros2_control index 16
  ghand_comm::JointId::LF_MCP,          // little_mcp,    ros2_control index 17
};

// ros2_control index in the 18-joint canonical order for each active joint above.
const std::vector<size_t> kEthercatActiveUrdfIndices = {
  1, 2, 3, 4, 6, 7, 8, 10, 11, 13, 14, 16, 17,
};

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
  printf("=== ECAT init: using interface '%s'\n", impl_->ifname.c_str());

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
  // EtherCAT uses a fixed 13-active-joint layout in canonical order (no DIP joints).
  // Override any param-provided list so the PDO format matches the slave.
  impl_->valid_joints = kEthercatActiveJoints;

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
  impl_->rxpdo_buffer[1] = 0x00;  // run (0x01 = stop)

  std::lock_guard<std::mutex> lock(impl_->rt_mutex);

  std::memset(&impl_->ctx, 0, sizeof(ecx_contextt));
  std::memset(impl_->io_map, 0, sizeof(impl_->io_map));

  if (ecx_init(&impl_->ctx, impl_->ifname.c_str()) <= 0)
  {
    printf("=== ECAT init: ecx_init failed on '%s'\n", impl_->ifname.c_str());
    return false;
  }
  printf("=== ECAT init: ecx_init OK\n");

  if (ecx_config_init(&impl_->ctx) <= 0 || impl_->ctx.slavecount <= 0)
  {
    printf("=== ECAT init: ecx_config_init failed or slavecount=%d\n", impl_->ctx.slavecount);
    return false;
  }
  printf("=== ECAT init: ecx_config_init OK, slavecount=%d\n", impl_->ctx.slavecount);

  if (ecx_config_map_group(&impl_->ctx, impl_->io_map, 0) <= 0)
  {
    printf("=== ECAT init: ecx_config_map_group failed\n");
    return false;
  }
  printf("=== ECAT init: ecx_config_map_group OK\n");

  impl_->mapping_done = 1;
  impl_->do_run = 1;
  ec_groupt * group = &impl_->ctx.grouplist[0];
  impl_->expected_wkc = (group->outputsWKC * 2) + group->inputsWKC;

  if (impl_->ctx.slavecount > 0)
  {
    printf("=== ECAT init: slavecount=%d Ibytes=%u Obytes=%u expected_wkc=%d\n",
           impl_->ctx.slavecount,
           impl_->ctx.slavelist[1].Ibytes,
           impl_->ctx.slavelist[1].Obytes,
           impl_->expected_wkc);
  }
  else
  {
    printf("=== ECAT init: slavecount=0, no slaves found\n");
  }

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
    printf("=== ECAT init: statecheck SAFE_OP failed\n");
    return false;
  }
  printf("=== ECAT init: SAFE_OP OK\n");

  // Start PDO threads before requesting OP so that cyclic process data is
  // already running while the slave transitions to OPERATIONAL.
  impl_->threads_started = true;
  impl_->rt_thread = std::thread([this]() { impl_->ecat_thread(); });
  impl_->check_thread = std::thread([this]() { impl_->ecat_check(); });

  // Wait briefly for the real-time thread to begin PDO exchange.
  osal_usleep(10000);

  impl_->ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
  ecx_writestate(&impl_->ctx, 0);

  // Poll OPERATIONAL state with a short retry loop.
  int op_retry = 0;
  constexpr int kMaxOpRetries = 5000;
  while (impl_->ctx.slavelist[0].state != EC_STATE_OPERATIONAL && op_retry < kMaxOpRetries)
  {
    if (ecx_statecheck(&impl_->ctx, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTRET) == EC_STATE_OPERATIONAL)
    {
      break;
    }
    osal_usleep(1000);
    ++op_retry;
  }

  if (impl_->ctx.slavelist[0].state != EC_STATE_OPERATIONAL)
  {
    printf("=== ECAT init: statecheck OPERATIONAL failed (state=0x%04x)\n",
           impl_->ctx.slavelist[0].state);
    close();
    return false;
  }
  printf("=== ECAT init: OPERATIONAL OK (%d ms)\n", op_retry);

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
  buffer[offset++] = 0x00;  // run flag (0x00 = run)

  for (const auto & joint_id : impl_->valid_joints)
  {
    float angle = 0.0f;
    uint8_t velocity = 100;
    uint8_t torque = 100;

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

  static int write_cnt = 0;
  if (++write_cnt % 100 == 0 || write_cnt == 1)
  {
    printf("=== write #%d: count=%zu\n", write_cnt, commands.size());
    for (size_t i = 0; i < commands.size(); ++i)
    {
      const auto & cmd = commands[i];
      printf("  [%zu] %-16s id=%2d pos_deg=%7.3f vel=%3d torque=%3d\n",
             i,
             joint_id_to_name(cmd.id).c_str(),
             static_cast<int>(cmd.id),
             cmd.position,
             cmd.velocity,
             cmd.torque);
    }
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
    // static int loop_cnt = 0;
    // if (++loop_cnt % 200 == 0)
    // {
    //   printf("=== ecat loop %d: slavecount=%d in_op=%d wkc=%d expected=%d pdo_size=%u\n",
    //          loop_cnt,
    //          ctx.slavecount,
    //          in_op,
    //          wkc,
    //          expected_wkc,
    //          ctx.slavecount > 0 ? ctx.slavelist[1].Ibytes : 0u);
    // }

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

      static bool pdo_dumped = false;
      if (!pdo_dumped && !pdo_copy.empty())
      {
        pdo_dumped = true;
        const size_t dump_len = std::min(pdo_copy.size(), size_t(64));
        printf("=== PDO size=%zu, first %zu bytes: ", pdo_copy.size(), dump_len);
        for (size_t i = 0; i < dump_len; ++i)
        {
          printf("%02x ", pdo_copy[i]);
        }
        printf("\n");
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

  // The slave reports 13 active joints in canonical order (no DIP joints). Expand
  // to the full 18-joint canonical order by placing active joints at their
  // URDF indices and leaving DIP joints at zero.
  constexpr size_t kUrdfJointCount = 18;
  out_data->joints.resize(kUrdfJointCount);

  for (size_t active_idx = 0; active_idx < valid_joints.size(); ++active_idx)
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

    const size_t urdf_idx = kEthercatActiveUrdfIndices[active_idx];
    JointState js;
    js.status = joint_state;
    js.error = joint_error;
    js.position = joint_angle * (180.0f / static_cast<float>(kPi));
    js.velocity = static_cast<float>(static_cast<int8_t>(joint_velocity));
    (void)joint_torque;
    out_data->joints[urdf_idx] = js;

    static bool first_joint_parsed = false;
    if (!first_joint_parsed)
    {
      first_joint_parsed = true;
      printf("=== first active joint: urdf_idx=%zu status=%u error=%u angle_deg=%.3f velocity=%d torque=%u\n",
             urdf_idx,
             js.status, js.error, js.position,
             static_cast<int>(static_cast<int8_t>(joint_velocity)),
             static_cast<unsigned>(joint_torque));
    }
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
