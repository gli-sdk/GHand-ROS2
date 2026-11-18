// Copyright (c) 2026 GHand ROS2 Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "ghand_hardware_interface/ghand_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>
#include <thread>

#include "ghand_hardware_interface/communication/communication.hpp"
#include "ghand_hardware_interface/communication/product_config.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ghand_hardware {

GHandHardwareInterface::~GHandHardwareInterface() {
  if (comm_) {
    if (active_) comm_->stop();
    comm_->close();
  }
}


hardware_interface::CallbackReturn GHandHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto mode = ghand_comm::parse_hand_control_mode(
      info_.hardware_parameters.count("control_mode")
          ? info_.hardware_parameters.at("control_mode") : "position");
  if (!mode) {
    RCLCPP_FATAL(rclcpp::get_logger("GHandHardwareInterface"),
                 "Invalid control_mode hardware parameter.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  hand_mode_ = *mode;
  const auto* product = ghand_comm::find_product_config(
      info_.hardware_parameters.count("product_name") ? info_.hardware_parameters.at("product_name") : "ghand5");
  if (!product || (product->lite1() && hand_mode_ == ghand_comm::HandControlMode::MIT)) {
    RCLCPP_FATAL(rclcpp::get_logger("GHandHardwareInterface"), "Unsupported product or control mode.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  lite1_ = product->lite1();
  hw_hand_state_.fill(std::numeric_limits<double>::quiet_NaN());

  const size_t joint_count = info_.joints.size();
  hw_joint_states_.resize(joint_count);
  hw_joint_commands_.resize(joint_count);
  hw_finger_tactile_data_.resize(info_.sensors.size());

  joint_id_map_.reserve(joint_count);
  command_joint_indices_.clear();
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto& joint = info_.joints[i];
    const auto id = ghand_comm::joint_name_to_id(joint.name);
    if (!id.has_value()) {
      RCLCPP_FATAL(rclcpp::get_logger("GHandHardwareInterface"),
                   "Joint '%s' has no matching SDK joint id.",
                   joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint_id_map_.push_back(id.value());
    if (!joint.command_interfaces.empty()) {
      if (std::find(product->active_joints.begin(), product->active_joints.end(), *id) == product->active_joints.end()) {
        RCLCPP_FATAL(rclcpp::get_logger("GHandHardwareInterface"), "Passive joint declares command interfaces.");
        return hardware_interface::CallbackReturn::ERROR;
      }
      command_joint_indices_.push_back(i);
    }
  }

  // Validate the selected product joint set exactly once. A mismatch
  // between URDF/xacro and the canonical JointId enum means motor commands
  // would be sent to the wrong joints.
  {
    std::vector<bool> seen(static_cast<size_t>(ghand_comm::JointId::NUM_JOINTS),
                           false);
    for (const auto id : joint_id_map_) {
      const size_t idx = static_cast<size_t>(id);
      if (idx >= seen.size()) {
        RCLCPP_FATAL(rclcpp::get_logger("GHandHardwareInterface"),
                     "Unexpected JointId %zu found in URDF joint mapping.",
                     idx);
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (seen[idx]) {
        RCLCPP_FATAL(rclcpp::get_logger("GHandHardwareInterface"),
                     "Duplicate JointId %zu in URDF joint mapping.", idx);
        return hardware_interface::CallbackReturn::ERROR;
      }
      seen[idx] = true;
    }
    for (size_t i = 0; i < seen.size(); ++i) {
      const bool expected = std::find(product->joints.begin(), product->joints.end(),
          static_cast<ghand_comm::JointId>(i)) != product->joints.end();
      if (seen[i] != expected) {
        RCLCPP_FATAL(
            rclcpp::get_logger("GHandHardwareInterface"),
            "Product joint set mismatch at JointId %zu (%s).", i,
            ghand_comm::joint_id_to_name(static_cast<ghand_comm::JointId>(i))
                .c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  if (command_joint_indices_.size() != product->active_joints.size()) {
    RCLCPP_FATAL(rclcpp::get_logger("GHandHardwareInterface"), "Product command joint set is incomplete.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Diagnostic: print the order in which ros2_control parsed the joints and the
  // resulting JointId mapping. This helps detect mismatches between URDF/xacro
  // order and the canonical order used by the communication backend.
  {
    std::string names;
    std::string ids;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      if (i > 0) {
        names += ", ";
        ids += ", ";
      }
      names += info_.joints[i].name;
      ids += std::to_string(static_cast<int>(joint_id_map_[i]));
    }
    RCLCPP_DEBUG(rclcpp::get_logger("GHandHardwareInterface"),
                "Parsed %zu joints in order: [%s] -> JointIds [%s]",
                info_.joints.size(), names.c_str(), ids.c_str());
  }

  if (info_.hardware_parameters.count("read_timeout_ms")) {
    try {
      read_timeout_ms_ =
          std::stod(info_.hardware_parameters.at("read_timeout_ms"));
    } catch (...) {
      RCLCPP_ERROR(rclcpp::get_logger("GHandHardwareInterface"), "Invalid read_timeout_ms parameter.");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  if (!std::isfinite(read_timeout_ms_) || read_timeout_ms_ <= 0) {
    RCLCPP_ERROR(rclcpp::get_logger("GHandHardwareInterface"), "read_timeout_ms must be finite and positive.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo& joint : info_.joints) {
    // Passive/state-only joints are allowed; they simply won't export command
    // interfaces or receive write() commands. All joints must still provide
    // state interfaces.
    if (!joint.command_interfaces.empty()) {
      for (const auto& ci : joint.command_interfaces) {
        if (ci.name != hardware_interface::HW_IF_POSITION &&
            ci.name != "velocity_percent" &&
            ci.name != "current_percent" && ci.name != "control_mode" &&
            ci.name != "loop_type" && ci.name != "control_state" &&
            ci.name != "kp" && ci.name != "kd") {
          RCLCPP_FATAL(rclcpp::get_logger("GHandHardwareInterface"),
                       "Joint '%s' has unsupported command interface '%s'.",
                       joint.name.c_str(), ci.name.c_str());
          return hardware_interface::CallbackReturn::ERROR;
        }
      }
    }

    if (joint.state_interfaces.empty()) {
      RCLCPP_FATAL(rclcpp::get_logger("GHandHardwareInterface"),
                   "Joint '%s' has no state interfaces.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const bool has_position = std::any_of(joint.state_interfaces.begin(), joint.state_interfaces.end(),
        [](const auto& si) { return si.name == hardware_interface::HW_IF_POSITION; });
    if (!has_position) return hardware_interface::CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < info_.sensors.size(); ++i) {
    const hardware_interface::ComponentInfo& sensor = info_.sensors[i];
    if (std::any_of(sensor.state_interfaces.begin(), sensor.state_interfaces.end(),
                    [](const auto& si) { return si.name == "hand_state"; })) {
      continue;
    }
    bool has_resultant_force_x = false;
    bool has_resultant_force_y = false;
    bool has_resultant_force_z = false;
    size_t distributed_point_count = 0;

    for (const auto& si : sensor.state_interfaces) {
      if (si.name == "resultant_force_x") {
        has_resultant_force_x = true;
      } else if (si.name == "resultant_force_y") {
        has_resultant_force_y = true;
      } else if (si.name == "resultant_force_z") {
        has_resultant_force_z = true;
      } else if (si.name.rfind("distributed_force_", 0) == 0 &&
                 si.name.size() > 2 &&
                 si.name.substr(si.name.size() - 2) == "_x") {
        ++distributed_point_count;
      }
    }

    if (!has_resultant_force_x || !has_resultant_force_y ||
        !has_resultant_force_z) {
      RCLCPP_FATAL(
          rclcpp::get_logger("GHandHardwareInterface"),
          "Sensor '%s' is missing required resultant_force state interfaces.",
          sensor.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    hw_finger_tactile_data_[i].distributed_forces.resize(
        distributed_point_count);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GHandHardwareInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_DEBUG(rclcpp::get_logger("GHandHardwareInterface"),
              "Configuring ...please wait...");

  auto it = info_.hardware_parameters.find("communication_type");
  if (it == info_.hardware_parameters.end() || it->second.empty()) {
    RCLCPP_FATAL(rclcpp::get_logger("GHandHardwareInterface"),
                 "Missing 'communication_type' hardware parameter.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  comm_ = ghand_comm::create_communication(it->second);
  if (!comm_) {
    RCLCPP_FATAL(rclcpp::get_logger("GHandHardwareInterface"),
                 "Unknown communication_type '%s'.", it->second.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::unordered_map<std::string, std::string> params(
      info_.hardware_parameters.begin(), info_.hardware_parameters.end());

  std::string joint_names;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (i > 0) joint_names += ",";
    joint_names += info_.joints[i].name;
  }
  params["joint_names"] = joint_names;

  active_ = false;
  feedback_wait_started_ = std::chrono::steady_clock::now();
  if (!comm_->init(params)) {
    RCLCPP_FATAL(rclcpp::get_logger("GHandHardwareInterface"),
                 "Failed to initialize %s communication backend.",
                 comm_->get_name());
    comm_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("GHandHardwareInterface"),
              "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
GHandHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    for (const auto& si : info_.joints[i].state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION) {
        state_interfaces.emplace_back(info_.joints[i].name,
                                      hardware_interface::HW_IF_POSITION,
                                      &hw_joint_states_[i].position);
      } else if (si.name == "velocity_percent") {
        state_interfaces.emplace_back(info_.joints[i].name,
                                      "velocity_percent",
                                      &hw_joint_states_[i].velocity_percent);
      } else if (si.name == "current_percent") {
        state_interfaces.emplace_back(info_.joints[i].name, si.name,
                                      &hw_joint_states_[i].current_percent);
      } else if (si.name == "control_mode") {
        state_interfaces.emplace_back(info_.joints[i].name, si.name,
                                      &hw_joint_states_[i].control_mode);
      } else if (si.name == "loop_type") {
        state_interfaces.emplace_back(info_.joints[i].name, si.name,
                                      &hw_joint_states_[i].loop_type);
      } else if (si.name == "state") {
        state_interfaces.emplace_back(info_.joints[i].name, "state",
                                      &hw_joint_states_[i].status);
      } else if (si.name == "error") {
        state_interfaces.emplace_back(info_.joints[i].name, "error",
                                      &hw_joint_states_[i].error);
      }
    }
  }

  for (size_t i = 0; i < info_.sensors.size(); ++i) {
    const auto& sensor = info_.sensors[i];
    if (std::any_of(sensor.state_interfaces.begin(), sensor.state_interfaces.end(),
                    [](const auto& si) { return si.name == "hand_state"; })) {
      for (const auto& si : sensor.state_interfaces) {
        size_t index = 0;
        if (si.name == "hand_state") index = 0;
        else if (si.name == "temperature_raw") index = 1;
        else {
          bool found = false;
          for (size_t j = 0; j < 24; ++j) {
            const std::string name = j < 6 ? "warning_" + std::to_string(j)
                : j < 18 ? "fault_" + std::to_string(j - 6)
                         : "critical_" + std::to_string(j - 18);
            if (si.name == name) { index = j + 2; found = true; break; }
          }
          if (!found) continue;
        }
        state_interfaces.emplace_back(sensor.name, si.name, &hw_hand_state_[index]);
      }
      continue;
    }
    state_interfaces.emplace_back(
        info_.sensors[i].name, "resultant_force_x",
        &hw_finger_tactile_data_[i].resultant_force.x);
    state_interfaces.emplace_back(
        info_.sensors[i].name, "resultant_force_y",
        &hw_finger_tactile_data_[i].resultant_force.y);
    state_interfaces.emplace_back(
        info_.sensors[i].name, "resultant_force_z",
        &hw_finger_tactile_data_[i].resultant_force.z);

    for (size_t j = 0; j < hw_finger_tactile_data_[i].distributed_forces.size();
         ++j) {
      const std::string base_name = "distributed_force_" + std::to_string(j);
      state_interfaces.emplace_back(
          info_.sensors[i].name, base_name + "_x",
          &hw_finger_tactile_data_[i].distributed_forces[j].x);
      state_interfaces.emplace_back(
          info_.sensors[i].name, base_name + "_y",
          &hw_finger_tactile_data_[i].distributed_forces[j].y);
      state_interfaces.emplace_back(
          info_.sensors[i].name, base_name + "_z",
          &hw_finger_tactile_data_[i].distributed_forces[j].z);
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
GHandHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t idx : command_joint_indices_) {
    for (const auto& ci : info_.joints[idx].command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_POSITION) {
        command_interfaces.emplace_back(info_.joints[idx].name,
                                        hardware_interface::HW_IF_POSITION,
                                        &hw_joint_commands_[idx].position);
      } else if (ci.name == "velocity_percent") {
        command_interfaces.emplace_back(info_.joints[idx].name,
                                        "velocity_percent",
                                        &hw_joint_commands_[idx].velocity_percent);
      } else if (ci.name == "current_percent") {
        command_interfaces.emplace_back(info_.joints[idx].name, ci.name,
                                        &hw_joint_commands_[idx].current_percent);
      } else if (ci.name == "control_mode") {
        command_interfaces.emplace_back(info_.joints[idx].name, ci.name,
                                        &hw_joint_commands_[idx].control_mode);
      } else if (ci.name == "loop_type") {
        command_interfaces.emplace_back(info_.joints[idx].name, ci.name,
                                        &hw_joint_commands_[idx].loop_type);
      } else if (ci.name == "control_state") {
        command_interfaces.emplace_back(info_.joints[idx].name, ci.name,
                                        &hw_joint_commands_[idx].control_state);
      } else if (ci.name == "kp") {
        command_interfaces.emplace_back(info_.joints[idx].name, ci.name,
                                        &hw_joint_commands_[idx].kp);
      } else if (ci.name == "kd") {
        command_interfaces.emplace_back(info_.joints[idx].name, ci.name,
                                        &hw_joint_commands_[idx].kd);

      }
    }
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn GHandHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_DEBUG(rclcpp::get_logger("GHandHardwareInterface"),
              "Activating ...please wait...");

  if (!comm_) return hardware_interface::CallbackReturn::ERROR;
  active_ = false;
  // Never seed a controller with fabricated zero feedback. Wait only during
  // activation; the periodic read path remains a cache read.
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double, std::milli>(read_timeout_ms_);
  bool ready = false;
  do {
    ghand_comm::CommunicationData data;
    if (comm_->read(data) && data.valid &&
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - data.received).count()
            <= read_timeout_ms_) {
      ready = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  } while (std::chrono::steady_clock::now() < deadline);
  if (!ready || read(rclcpp::Time(0), rclcpp::Duration(0, 0)) != hardware_interface::return_type::OK) {
    RCLCPP_ERROR(rclcpp::get_logger("GHandHardwareInterface"), "Activation requires fresh SDK joint feedback.");
    comm_->stop();
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (auto& command : hw_joint_commands_) command = JointCommand{};
  active_ = true;

  RCLCPP_INFO(rclcpp::get_logger("GHandHardwareInterface"),
              "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GHandHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("GHandHardwareInterface"),
              "Deactivating ...please wait...");

  active_ = false;
  const bool stopped = !comm_ || comm_->stop();

  RCLCPP_INFO(rclcpp::get_logger("GHandHardwareInterface"),
              "Deactivation complete; SDK stop %s.", stopped ? "succeeded" : "failed");

  return stopped ? hardware_interface::CallbackReturn::SUCCESS : hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn GHandHardwareInterface::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_ERROR(rclcpp::get_logger("GHandHardwareInterface"),
               "Entering error state; stopping the hand through SDK.");

  active_ = false;
  const bool stopped = !comm_ || comm_->stop();
  if (comm_) comm_->close();
  return stopped ? hardware_interface::CallbackReturn::SUCCESS : hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn GHandHardwareInterface::on_cleanup(const rclcpp_lifecycle::State&) {
  active_ = false;
  if (comm_) comm_->close();
  comm_.reset();
  for (auto& command : hw_joint_commands_) command = JointCommand{};
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GHandHardwareInterface::on_shutdown(const rclcpp_lifecycle::State& state) {
  const bool stopped = !comm_ || comm_->stop();
  on_cleanup(state);
  return stopped ? hardware_interface::CallbackReturn::SUCCESS : hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::return_type GHandHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {

  if (!comm_) {
    return hardware_interface::return_type::ERROR;
  }

  ghand_comm::CommunicationData data;
  const bool ok = comm_->read(data);

  if (ok && data.valid) {


    constexpr double kPi = 3.14159265358979323846;
    constexpr double kDegToRad = kPi / 180.0;

    // SDK V3.1.0 feedback identifies each joint explicitly; vectors may be
    // sparse or arrive in a different order from ros2_control resources.
    for (const auto& joint : data.device.joints) {
      const auto found = std::find(joint_id_map_.begin(), joint_id_map_.end(), joint.id);
      if (found == joint_id_map_.end()) continue;
      const size_t i = static_cast<size_t>(found - joint_id_map_.begin());
      hw_joint_states_[i].position = static_cast<double>(joint.angle) * kDegToRad;
      hw_joint_states_[i].velocity_percent = static_cast<double>(joint.velocity);
      hw_joint_states_[i].current_percent = joint.current;
      hw_joint_states_[i].control_mode = data.joint_control_info_available
          ? static_cast<uint8_t>(joint.control_mode) : std::numeric_limits<double>::quiet_NaN();
      hw_joint_states_[i].loop_type = !lite1_ && data.joint_control_info_available
          ? static_cast<uint8_t>(joint.loop_type) : std::numeric_limits<double>::quiet_NaN();
      hw_joint_states_[i].status = static_cast<uint8_t>(joint.state);
      hw_joint_states_[i].error = static_cast<uint8_t>(joint.error);
    }

    if (data.hand_state_valid) {
      hw_hand_state_[0] = static_cast<uint16_t>(data.device.hand.joints_state);
      hw_hand_state_[1] = data.device.hand.temperature;
      std::copy(data.device.hand.error_warn.begin(), data.device.hand.error_warn.end(),
                hw_hand_state_.begin() + 2);
      std::copy(data.device.hand.error_fault.begin(), data.device.hand.error_fault.end(),
                hw_hand_state_.begin() + 8);
      std::copy(data.device.hand.error_critical.begin(), data.device.hand.error_critical.end(),
                hw_hand_state_.begin() + 20);
    } else {
      hw_hand_state_.fill(std::numeric_limits<double>::quiet_NaN());
    }

    for (size_t i = 0; i < hw_finger_tactile_data_.size(); ++i) {
      if (std::any_of(info_.sensors[i].state_interfaces.begin(),
                      info_.sensors[i].state_interfaces.end(),
                      [](const auto& si) { return si.name == "hand_state"; })) continue;
      const auto sensor_id = ghand_comm::tactile_sensor_id(info_.sensors[i].name);
      const auto found = sensor_id ? data.device.tactile.find(*sensor_id) : data.device.tactile.end();
      const ghand_comm::TactileData* tactile =
          found != data.device.tactile.end() ? &found->second : nullptr;
      const auto component = [](const std::vector<float>* values, size_t index) {
        return values && index < values->size() ? static_cast<double>((*values)[index]) : 0.0;
      };
      auto& target = hw_finger_tactile_data_[i];
      const auto* resultant = tactile ? &tactile->resultant_force : nullptr;
      target.resultant_force.x = component(resultant, 0);
      target.resultant_force.y = component(resultant, 1);
      target.resultant_force.z = component(resultant, 2);
      const auto* distributed = tactile ? &tactile->distributed_force : nullptr;
      for (size_t j = 0; j < target.distributed_forces.size(); ++j) {
        target.distributed_forces[j].x = component(distributed, j * 3);
        target.distributed_forces[j].y = component(distributed, j * 3 + 1);
        target.distributed_forces[j].z = component(distributed, j * 3 + 2);
      }
    }
  }

  const auto received = ok && data.valid ? data.received : feedback_wait_started_;
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - received).count();
  if (elapsed_ms > read_timeout_ms_) {
    static rclcpp::Clock throttle_clock(RCL_STEADY_TIME);
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("GHandHardwareInterface"),
                          throttle_clock, 1000,
                          "Hardware read timeout: no valid data for %.1f ms.",
                          read_timeout_ms_);
    if (active_) comm_->stop();
    active_ = false;
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GHandHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (!comm_) return hardware_interface::return_type::ERROR;
  if (!active_) return hardware_interface::return_type::OK;

  std::vector<ghand_comm::JointCommand> commands;
  commands.reserve(command_joint_indices_.size());
  for (size_t idx : command_joint_indices_) {
    const auto& input = hw_joint_commands_[idx];
    if (std::isnan(input.position) && std::isnan(input.velocity_percent) && std::isnan(input.current_percent) &&
        std::isnan(input.control_state)) continue;

    ghand_comm::JointCommand cmd;
    if (!ghand_comm::make_joint_command(input, hand_mode_,
                                        hw_joint_states_[idx].position, &cmd)) {
      RCLCPP_ERROR(rclcpp::get_logger("GHandHardwareInterface"),
                   "Invalid command for joint '%s'; refusing hardware write.",
                   info_.joints[idx].name.c_str());
      active_ = false;
      comm_->stop();
      return hardware_interface::return_type::ERROR;
    }
    cmd.id = joint_id_map_[idx];
    commands.push_back(cmd);
  }

  if (!comm_->write(commands)) {
    RCLCPP_ERROR(rclcpp::get_logger("GHandHardwareInterface"),
                 "Hardware write failed on %s backend; requesting SDK stop.",
                 comm_->get_name());
    active_ = false;
    comm_->stop();
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace ghand_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ghand_hardware::GHandHardwareInterface,
                       hardware_interface::SystemInterface)
