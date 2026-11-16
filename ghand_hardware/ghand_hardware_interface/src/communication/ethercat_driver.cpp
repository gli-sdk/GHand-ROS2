// Copyright (c) 2026 GHand ROS2 Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "ghand_hardware_interface/communication/ethercat_driver.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ghand_hardware_interface/communication/modbus_codec.hpp"
#include "ghand_hardware_interface/communication/product_config.hpp"
#include "ghand_hardware_interface/communication/util.hpp"
#include "rclcpp/logging.hpp"

#ifdef GHAND_HAS_SOEM
#include <soem/soem.h>
#endif

namespace ghand_comm {

namespace {

constexpr int kEcTimeoutMon = 500;
constexpr int kMaxOpRetries = 5000;

}  // namespace

struct EthercatDriver::Impl {
  const ProductConfig* product = find_product_config("ghand5");
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
  std::vector<uint8_t> rxpdo_buffer;

  std::mutex rt_mutex;
  std::mutex data_mutex;

  std::string ifname;
  std::vector<JointId> valid_joints;
  std::vector<std::pair<std::string, size_t>> tactile_regions;
  CommunicationData latest_data;
  std::map<JointId, JointCommand> last_commands;

  std::atomic<bool> threads_started{false};
  std::thread rt_thread;
  std::thread check_thread;

  void ecat_thread();
  void ecat_check();
  void parse_and_notify(const uint8_t* data, size_t size);
  void parse_hand_and_joints(const uint8_t* data, size_t size, size_t* offset,
                             CommunicationData* out_data) const;
  void parse_tactile_data(const uint8_t* data, size_t size, size_t* offset,
                          CommunicationData* out_data) const;
};

EthercatDriver::EthercatDriver() : impl_(std::make_unique<Impl>()) {}

EthercatDriver::~EthercatDriver() { close(); }

const char* EthercatDriver::get_name() const { return "ethercat"; }

bool EthercatDriver::init(
    const std::unordered_map<std::string, std::string>& params) {
  const auto mode = parse_hand_control_mode(
      params.count("control_mode") ? params.at("control_mode") : "position");
  if (!mode) return false;
  hand_mode_ = *mode;
  impl_->product = find_product_config(params.count("product_name") ? params.at("product_name") : "ghand5");
  if (!impl_->product || (impl_->product->lite1() && hand_mode_ == HandControlMode::MIT)) return false;
#ifndef GHAND_HAS_SOEM
  (void)params;
  return false;
#else
  auto it = params.find("ifname");
  if (it == params.end() || it->second.empty()) {
    RCLCPP_ERROR(
        rclcpp::get_logger("ethercat"),
        "EtherCAT 'ifname' parameter is missing or empty. "
        "Provide it via launch arg: ifname:=<interface>");
    return false;
  }
  impl_->ifname = it->second;
  RCLCPP_INFO(rclcpp::get_logger("ethercat"),
              "=== ECAT init: using interface '%s'", impl_->ifname.c_str());

  auto jt = params.find("joint_names");
  if (jt != params.end()) {
    for (const auto& name : split(jt->second, ',')) {
      const auto id = joint_name_to_id(name);
      if (id.has_value()) {
        impl_->valid_joints.push_back(id.value());
      }
    }
  }
  impl_->valid_joints = impl_->product->active_joints;

  auto tt = params.find("tactile_region_counts");
  if (tt != params.end()) {
    impl_->tactile_regions = parse_tactile_regions(tt->second);
  }

  impl_->rxpdo_buffer.assign(impl_->product->rxpdo_size(), 0);
  impl_->rxpdo_buffer[1] = 0x00;  // disabled until the first write()

  std::lock_guard<std::mutex> lock(impl_->rt_mutex);

  std::memset(&impl_->ctx, 0, sizeof(ecx_contextt));
  std::memset(impl_->io_map, 0, sizeof(impl_->io_map));

  if (ecx_init(&impl_->ctx, impl_->ifname.c_str()) <= 0) {
    RCLCPP_INFO(rclcpp::get_logger("ethercat"),
                "=== ECAT init: ecx_init failed on '%s'",
                impl_->ifname.c_str());
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("ethercat"), "=== ECAT init: ecx_init OK");

  if (ecx_config_init(&impl_->ctx) <= 0 || impl_->ctx.slavecount <= 0) {
    RCLCPP_INFO(rclcpp::get_logger("ethercat"),
                "=== ECAT init: ecx_config_init failed or slavecount=%d",
                impl_->ctx.slavecount);
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("ethercat"),
              "=== ECAT init: ecx_config_init OK, slavecount=%d",
              impl_->ctx.slavecount);

  if (ecx_config_map_group(&impl_->ctx, impl_->io_map, 0) <= 0) {
    RCLCPP_INFO(rclcpp::get_logger("ethercat"),
                "=== ECAT init: ecx_config_map_group failed");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("ethercat"),
              "=== ECAT init: ecx_config_map_group OK");

  if (impl_->ctx.slavelist[1].Obytes != impl_->rxpdo_buffer.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("ethercat"),
                 "PDO output size mismatch for %s: expected %zu, received %u",
                 impl_->product->name.c_str(), impl_->rxpdo_buffer.size(),
                 impl_->ctx.slavelist[1].Obytes);
    return false;
  }

  impl_->mapping_done = 1;
  impl_->do_run = 1;
  ec_groupt* group = &impl_->ctx.grouplist[0];
  impl_->expected_wkc = (group->outputsWKC * 2) + group->inputsWKC;

  if (impl_->ctx.slavecount > 0) {
    RCLCPP_INFO(
        rclcpp::get_logger("ethercat"),
        "=== ECAT init: slavecount=%d Ibytes=%u Obytes=%u expected_wkc=%d",
        impl_->ctx.slavecount, impl_->ctx.slavelist[1].Ibytes,
        impl_->ctx.slavelist[1].Obytes, impl_->expected_wkc);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("ethercat"),
                "=== ECAT init: slavecount=0, no slaves found");
  }

  ecx_configdc(&impl_->ctx);

  for (int si = 1; si <= impl_->ctx.slavecount; ++si) {
    if (impl_->ctx.slavelist[si].CoEdetails > 0) {
      ecx_slavembxcyclic(&impl_->ctx, si);
    }
  }

  if (ecx_statecheck(&impl_->ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE) !=
      EC_STATE_SAFE_OP) {
    RCLCPP_INFO(rclcpp::get_logger("ethercat"),
                "=== ECAT init: statecheck SAFE_OP failed");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("ethercat"), "=== ECAT init: SAFE_OP OK");

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
  while (impl_->ctx.slavelist[0].state != EC_STATE_OPERATIONAL &&
         op_retry < kMaxOpRetries) {
    if (ecx_statecheck(&impl_->ctx, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTRET) ==
        EC_STATE_OPERATIONAL) {
      break;
    }
    osal_usleep(1000);
    ++op_retry;
  }

  if (impl_->ctx.slavelist[0].state != EC_STATE_OPERATIONAL) {
    RCLCPP_INFO(rclcpp::get_logger("ethercat"),
                "=== ECAT init: statecheck OPERATIONAL failed (state=0x%04x);",
                impl_->ctx.slavelist[0].state);
    close();
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("ethercat"),
              "=== ECAT init: OPERATIONAL OK (%d ms);", op_retry);

  impl_->in_op = 1;
  impl_->is_connected = true;
  RCLCPP_INFO(rclcpp::get_logger("ethercat"),
              "[ethercat] initialized: ifname=%s", impl_->ifname.c_str());
  return true;
#endif
}

void EthercatDriver::close() {
#ifdef GHAND_HAS_SOEM
  impl_->threads_started = false;
  impl_->do_run = 0;
  impl_->in_op = 0;

  if (impl_->rt_thread.joinable()) impl_->rt_thread.join();
  if (impl_->check_thread.joinable()) impl_->check_thread.join();

  {
    std::lock_guard<std::mutex> lock(impl_->rt_mutex);
    if (impl_->ctx.slavecount > 0) {
      impl_->ctx.slavelist[0].state = EC_STATE_INIT;
      ecx_writestate(&impl_->ctx, 0);
    }
  }
#endif
  impl_->is_connected = false;
  impl_->last_commands.clear();
}

bool EthercatDriver::read(CommunicationData& out_data) {
  std::lock_guard<std::mutex> lock(impl_->data_mutex);
  out_data = impl_->latest_data;
  return impl_->is_connected && out_data.valid;
}

bool EthercatDriver::write(const std::vector<JointCommand>& commands) {
#ifdef GHAND_HAS_SOEM
  if (!impl_->is_connected) return false;

  for (const auto& command : commands) impl_->last_commands[command.id] = command;
  const auto buffer = encode_ethercat_command(hand_mode_, impl_->valid_joints,
      impl_->last_commands, impl_->product->command_register_count);
  if (buffer.size() != impl_->rxpdo_buffer.size()) return false;
  {
    std::lock_guard<std::mutex> lock(impl_->rt_mutex);
    std::copy(buffer.begin(), buffer.end(), impl_->rxpdo_buffer.begin());
  }

  return true;
#else
  (void)commands;
  return false;
#endif
}

#ifdef GHAND_HAS_SOEM

void EthercatDriver::Impl::ecat_thread() {
  ec_timet ts;
  int ht;

  while (!mapping_done) {
    osal_usleep(500);
  }

  osal_get_monotonic_time(&ts);
  ht = (ts.tv_nsec / 1000000) + 1;
  ts.tv_nsec = ht * 1000000;

  {
    std::lock_guard<std::mutex> lock(rt_mutex);
    if (ctx.slavecount > 0) {
      const size_t output_size =
          rxpdo_buffer.size();
      std::memcpy(ctx.slavelist[1].outputs, rxpdo_buffer.data(), output_size);
    }
    ecx_send_processdata(&ctx);
  }

  while (threads_started) {
    // static int loop_cnt = 0;
    // if (++loop_cnt % 200 == 0)
    // {
    //   RCLCPP_INFO(rclcpp::get_logger("ethercat"), "=== ecat loop %d:
    //   slavecount=%d in_op=%d wkc=%d expected=%d pdo_size=%u",    // loop_cnt,
    //   //          ctx.slavecount,    //          in_op,    //          wkc,
    //   //          expected_wkc,    //          ctx.slavecount > 0 ?
    //   ctx.slavelist[1].Ibytes : 0u);
    // }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    if (do_run > 0) {
      std::vector<uint8_t> pdo_copy;

      {
        std::lock_guard<std::mutex> lock(rt_mutex);
        wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);

        if (wkc >= expected_wkc && in_op && ctx.slavecount > 0) {
          uint8_t* inputs = ctx.slavelist[1].inputs;
          size_t pdo_size = ctx.slavelist[1].Ibytes;
          if (pdo_size > 0) {
            pdo_copy.assign(inputs, inputs + pdo_size);
          }
        }
      }

      // static bool pdo_dumped = false;
      // if (!pdo_dumped && !pdo_copy.empty())
      // {
      //   pdo_dumped = true;
      //   const size_t dump_len = std::min(pdo_copy.size(), size_t(64));
      //   RCLCPP_INFO(rclcpp::get_logger("ethercat"), "=== PDO size=%zu, first
      //   %zu bytes: ", pdo_copy.size();, dump_len); for (size_t i = 0; i <
      //   dump_len; ++i)
      //   {
      //     RCLCPP_INFO(rclcpp::get_logger("ethercat"), "%02x ", pdo_copy[i]);
      //   }
      //   RCLCPP_INFO(rclcpp::get_logger("ethercat"), "");
      // }

      if (!pdo_copy.empty()) {
        parse_and_notify(pdo_copy.data(), pdo_copy.size());
      }

      if (wkc != expected_wkc) {
        ++wkc_check;
      } else {
        wkc_check = 0;
      }

      {
        std::lock_guard<std::mutex> lock(rt_mutex);
        if (ctx.slavecount > 0) {
          const size_t output_size =
              rxpdo_buffer.size();
          std::memcpy(ctx.slavelist[1].outputs, rxpdo_buffer.data(), output_size);
        }
        ecx_send_processdata(&ctx);
      }
    }
  }
}

void EthercatDriver::Impl::ecat_check() {
  while (threads_started) {
    osal_usleep(50000);

    if (in_op && (wkc_check > 2 || ctx.grouplist[0].docheckstate)) {
      std::unique_lock<std::mutex> lock(rt_mutex, std::try_to_lock);
      if (!lock.owns_lock()) continue;

      ctx.grouplist[0].docheckstate = FALSE;
      ecx_readstate(&ctx);
      for (int slaveix = 1; slaveix <= ctx.slavecount; ++slaveix) {
        ec_slavet* slave = &ctx.slavelist[slaveix];
        if ((slave->group == 0) && (slave->state != EC_STATE_OPERATIONAL)) {
          ctx.grouplist[0].docheckstate = TRUE;
          if (slave->state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
            slave->state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
            ecx_writestate(&ctx, slaveix);
          } else if (slave->state == EC_STATE_SAFE_OP) {
            slave->state = EC_STATE_OPERATIONAL;
            if (slave->mbxhandlerstate == ECT_MBXH_LOST) {
              slave->mbxhandlerstate = ECT_MBXH_CYCLIC;
            }
            ecx_writestate(&ctx, slaveix);
          } else if (slave->state > EC_STATE_NONE) {
            if (ecx_reconfig_slave(&ctx, slaveix, kEcTimeoutMon) >=
                EC_STATE_PRE_OP) {
              slave->islost = FALSE;
            }
          } else if (!slave->islost) {
            ecx_statecheck(&ctx, slaveix, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (slave->state == EC_STATE_NONE) {
              slave->islost = TRUE;
              threads_started = false;
              is_connected = false;
              slave->mbxhandlerstate = ECT_MBXH_LOST;
              if (slave->Ibytes) {
                std::memset(slave->inputs, 0x00, slave->Ibytes);
              }
            }
          }
        }
        if (slave->islost) {
          if (slave->state <= EC_STATE_INIT) {
            if (ecx_recover_slave(&ctx, slaveix, kEcTimeoutMon)) {
              slave->islost = FALSE;
            }
          } else {
            slave->islost = FALSE;
          }
        }
      }
      wkc_check = 0;
    }
  }
}

namespace {

Force parse_resultant_force(const uint8_t* data) {
  Force resultant;
  if (data != nullptr) {
    const int16_t raw_x = static_cast<int16_t>(data[0] | (data[1] << 8));
    resultant.x = static_cast<float>(static_cast<int8_t>(raw_x & 0xFF)) * 0.1f;

    const int16_t raw_y = static_cast<int16_t>(data[2] | (data[3] << 8));
    resultant.y = static_cast<float>(static_cast<int8_t>(raw_y & 0xFF)) * 0.1f;

    const uint16_t raw_z = static_cast<uint16_t>(data[4] | (data[5] << 8));
    resultant.z = static_cast<float>(static_cast<uint8_t>(raw_z & 0xFF)) * 0.1f;
  }
  return resultant;
}

std::vector<Force> parse_sample_forces(const uint8_t* data, int sensor_count) {
  std::vector<Force> forces;
  forces.reserve(sensor_count);
  if (data != nullptr && sensor_count > 0) {
    for (int i = 0; i < sensor_count; ++i) {
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
    const uint8_t* data, size_t size, size_t* offset,
    CommunicationData* out_data) const {
  const size_t kHandHeaderBytes = product->hand_register_count * 2;
  const size_t fault_bytes = product->lite1() ? 6 : kHandFaultBytes;
  if (*offset + kHandHeaderBytes > size) return;

  uint16_t hand_state = 0;
  int16_t temperature = 0;

  std::memcpy(&hand_state, data + *offset, sizeof(hand_state));
  *offset += sizeof(hand_state);
  std::memcpy(out_data->hand_state.error_warn.data(), data + *offset,
              kHandWarningBytes);
  *offset += kHandWarningBytes;
  std::memcpy(out_data->hand_state.error_fault.data(), data + *offset,
              fault_bytes);
  *offset += fault_bytes;
  std::memcpy(out_data->hand_state.error_critical.data(), data + *offset,
              kHandCriticalBytes);
  *offset += kHandCriticalBytes;
  std::memcpy(&temperature, data + *offset, sizeof(temperature));
  *offset += sizeof(temperature);

  out_data->hand_state_valid = true;
  out_data->hand_state.state = hand_state;
  out_data->hand_state.temperature = temperature;

  // The slave reports the product joint order (including passive joints),
  // 10 bytes each: state, error, mode/loop, reserved, then
  // int16 angle (0.1 deg), velocity and current.
  const size_t kUrdfJointCount = product->joints.size();
  constexpr size_t kJointFeedbackBytes = 10;
  out_data->joints.resize(static_cast<size_t>(JointId::NUM_JOINTS));

  for (size_t urdf_idx = 0; urdf_idx < kUrdfJointCount; ++urdf_idx) {
    if (*offset + kJointFeedbackBytes > size) break;

    uint8_t joint_state = 0;
    uint8_t joint_error = 0;
    uint8_t joint_mode = 0;
    int16_t joint_angle = 0;
    int16_t joint_velocity = 0;
    int16_t joint_current = 0;

    std::memcpy(&joint_state, data + *offset, sizeof(joint_state));
    *offset += sizeof(joint_state);
    std::memcpy(&joint_error, data + *offset, sizeof(joint_error));
    *offset += sizeof(joint_error);
    std::memcpy(&joint_mode, data + *offset, sizeof(joint_mode));
    *offset += sizeof(joint_mode);
    *offset += 1;  // Reserved feedback byte.
    std::memcpy(&joint_angle, data + *offset, sizeof(joint_angle));
    *offset += sizeof(joint_angle);
    std::memcpy(&joint_velocity, data + *offset, sizeof(joint_velocity));
    *offset += sizeof(joint_velocity);
    std::memcpy(&joint_current, data + *offset, sizeof(joint_current));
    *offset += sizeof(joint_current);

    JointState js;
    js.status = joint_state;
    js.error = joint_error;
    js.mode = static_cast<uint8_t>(joint_mode & 0x0F);
    js.loop_type = static_cast<JointLoopType>((joint_mode >> 4) & 0x0F);
    js.control_info_valid = true;
    js.position = static_cast<float>(joint_angle) / 10.0f;
    js.velocity = static_cast<float>(joint_velocity);
    js.current = static_cast<float>(joint_current);
    if (product->lite1()) js.loop_type = JointLoopType::SEMI_CLOSED_LOOP;
    out_data->joints[static_cast<size_t>(product->joints[urdf_idx])] = js;
  }

  // Diagnostic: periodically dump the joint mapping as received from the slave.
  //   {
  //     static int parse_count = 0;
  //     if (++parse_count % 100 == 0 || parse_count == 1)
  //     {
  //       RCLCPP_INFO(rclcpp::get_logger("ethercat"), "===
  //       parse_hand_and_joints #%d (18 joints);", parse_count); for (size_t
  //       urdf_idx = 0; urdf_idx < kUrdfJointCount; ++urdf_idx)
  //       {
  //         const auto & js = out_data->joints[urdf_idx];
  //         RCLCPP_INFO(rclcpp::get_logger("ethercat"), "  [urdf %2zu] %-16s
  //         pos_deg=%7.3f",//                urdf_idx,//
  //         joint_id_to_name(static_cast<JointId>(urdf_idx);).c_str(),//
  //         js.position)
  //       }
  //     }
  //   }
}

void EthercatDriver::Impl::parse_tactile_data(
    const uint8_t* data, size_t size, size_t* offset,
    CommunicationData* out_data) const {
  if (*offset + 2 <= size) {
    const uint8_t sensor_state = data[*offset];
    const uint8_t sensor_error = data[*offset + 1];
    *offset += 2;
    (void)sensor_state;
    (void)sensor_error;
  }

  out_data->tactile_regions.reserve(tactile_regions.size());
  for (const auto& region : tactile_regions) {
    TactileRegion tr;
    tr.name = region.first;

    if (*offset + 6 <= size) {
      tr.resultant_force = parse_resultant_force(data + *offset);
      *offset += 6;
    }

    const int sample_size = static_cast<int>(region.second) * 3;
    if (region.second > 0 && *offset + sample_size <= size) {
      tr.distributed_forces =
          parse_sample_forces(data + *offset, static_cast<int>(region.second));
      *offset += sample_size;
    }

    out_data->tactile_regions.push_back(std::move(tr));
  }
}

void EthercatDriver::Impl::parse_and_notify(const uint8_t* data, size_t size) {
  if (data == nullptr) return;
  size_t expected = product->hand_register_count * 2 + product->joints.size() * 10;
  if (!tactile_regions.empty()) {
    expected += 2;
    for (const auto& region : tactile_regions) expected += 6 + region.second * 3;
  }
  if (size < expected) return;

  CommunicationData parsed;
  parsed.valid = true;
  size_t offset = 0;
  parse_hand_and_joints(data, size, &offset, &parsed);

  if (!tactile_regions.empty() && offset < size) {
    parse_tactile_data(data, size, &offset, &parsed);
  }

  {
    std::lock_guard<std::mutex> lock(data_mutex);
    latest_data = std::move(parsed);
  }
}

#endif  // GHAND_HAS_SOEM

}  // namespace ghand_comm
