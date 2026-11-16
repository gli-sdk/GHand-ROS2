// Copyright (c) 2026 GHand ROS2 Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "ghand_hardware_interface/ghand_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "ghand_hardware_interface/communication/communication.hpp"
#include "ghand_hardware_interface/communication/modbus_codec.hpp"
#include "ghand_hardware_interface/communication/product_config.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace ghand_hardware {

hardware_interface::CallbackReturn GHandHardwareInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
  if (hardware_interface::SystemInterface::on_init(params) !=
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
    RCLCPP_INFO(rclcpp::get_logger("GHandHardwareInterface"),
                "Parsed %zu joints in order: [%s] -> JointIds [%s]",
                info_.joints.size(), names.c_str(), ids.c_str());
  }

  if (info_.hardware_parameters.count("read_timeout_ms")) {
    try {
      read_timeout_ms_ =
          std::stod(info_.hardware_parameters.at("read_timeout_ms"));
    } catch (...) {
      RCLCPP_WARN(rclcpp::get_logger("GHandHardwareInterface"),
                  "Invalid read_timeout_ms parameter; using default %.1f ms.",
                  read_timeout_ms_);
    }
  }

  for (const hardware_interface::ComponentInfo& joint : info_.joints) {
    // Passive/state-only joints are allowed; they simply won't export command
    // interfaces or receive write() commands. All joints must still provide
    // state interfaces.
    if (!joint.command_interfaces.empty()) {
      for (const auto& ci : joint.command_interfaces) {
        if (ci.name != hardware_interface::HW_IF_POSITION &&
            ci.name != hardware_interface::HW_IF_VELOCITY &&
            ci.name != hardware_interface::HW_IF_EFFORT &&
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

    bool has_position = false;
    bool has_velocity = false;
    bool has_effort = false;
    for (const auto& si : joint.state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION) {
        has_position = true;
      } else if (si.name == hardware_interface::HW_IF_VELOCITY) {
        has_velocity = true;
      } else if (si.name == hardware_interface::HW_IF_EFFORT) {
        has_effort = true;
      }
    }

    if (!has_position) {
      RCLCPP_FATAL(rclcpp::get_logger("GHandHardwareInterface"),
                   "Joint '%s' is missing required state interface '%s'.",
                   joint.name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!has_velocity) {
      RCLCPP_FATAL(rclcpp::get_logger("GHandHardwareInterface"),
                   "Joint '%s' is missing required state interface '%s'.",
                   joint.name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!has_effort) {
      RCLCPP_FATAL(rclcpp::get_logger("GHandHardwareInterface"),
                   "Joint '%s' is missing required state interface '%s'.",
                   joint.name.c_str(), hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }
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
  RCLCPP_INFO(rclcpp::get_logger("GHandHardwareInterface"),
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
      } else if (si.name == hardware_interface::HW_IF_VELOCITY) {
        state_interfaces.emplace_back(info_.joints[i].name,
                                      hardware_interface::HW_IF_VELOCITY,
                                      &hw_joint_states_[i].velocity);
      } else if (si.name == hardware_interface::HW_IF_EFFORT) {
        state_interfaces.emplace_back(info_.joints[i].name,
                                      hardware_interface::HW_IF_EFFORT,
                                      &hw_joint_states_[i].effort);
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
      } else if (ci.name == hardware_interface::HW_IF_VELOCITY) {
        command_interfaces.emplace_back(info_.joints[idx].name,
                                        hardware_interface::HW_IF_VELOCITY,
                                        &hw_joint_commands_[idx].velocity);
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
      } else if (ci.name == hardware_interface::HW_IF_EFFORT) {
        command_interfaces.emplace_back(info_.joints[idx].name,
                                        hardware_interface::HW_IF_EFFORT,
                                        &hw_joint_commands_[idx].effort);
      }
    }
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn GHandHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("GHandHardwareInterface"),
              "Activating ...please wait...");

  for (auto& state : hw_joint_states_) {
    if (std::isnan(state.position)) {
      state.position = 0.0;
    }
    if (std::isnan(state.velocity)) {
      state.velocity = 0.0;
    }
    if (std::isnan(state.effort)) {
      state.effort = 0.0;
    }
  }

  // Leave command interfaces as NaN so write() can detect which ones the active
  // controller actually updates and fill the defaults for the current mode.

  last_successful_read_ = rclcpp::Time(0, 0);

  RCLCPP_INFO(rclcpp::get_logger("GHandHardwareInterface"),
              "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GHandHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("GHandHardwareInterface"),
              "Deactivating ...please wait...");

  if (comm_) {
    comm_->close();
  }

  RCLCPP_INFO(rclcpp::get_logger("GHandHardwareInterface"),
              "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GHandHardwareInterface::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_ERROR(rclcpp::get_logger("GHandHardwareInterface"),
               "Entering error state; freezing commands.");

  for (size_t i = 0; i < hw_joint_commands_.size(); ++i) {
    hw_joint_commands_[i].position = hw_joint_states_[i].position;
    hw_joint_commands_[i].velocity = hw_joint_states_[i].velocity;
    hw_joint_commands_[i].effort = hw_joint_states_[i].effort;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type GHandHardwareInterface::read(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
  if (last_successful_read_.nanoseconds() == 0) {
    last_successful_read_ = time;
  }

  if (!comm_) {
    return hardware_interface::return_type::ERROR;
  }

  ghand_comm::CommunicationData data;
  const bool ok = comm_->read(data);

  if (ok && data.valid) {
    last_successful_read_ = time;

    constexpr double kPi = 3.14159265358979323846;
    constexpr double kDegToRad = kPi / 180.0;

    for (size_t i = 0; i < hw_joint_states_.size() && i < data.joints.size();
         ++i) {
      // Map the hardware-state index (which follows the JointId / canonical
      // order used by the communication backend) into our local joint-state
      // buffer (which follows the order of info_.joints as parsed by
      // ros2_control).
      const size_t data_idx = static_cast<size_t>(joint_id_map_[i]);
      if (data_idx >= data.joints.size()) {
        continue;
      }
      hw_joint_states_[i].position =
          static_cast<double>(data.joints[data_idx].position) * kDegToRad;
      hw_joint_states_[i].velocity =
          static_cast<double>(data.joints[data_idx].velocity);
      hw_joint_states_[i].effort = 0.0;
      hw_joint_states_[i].current_percent = data.joints[data_idx].current;
      hw_joint_states_[i].control_mode = data.joints[data_idx].control_info_valid
          ? data.joints[data_idx].mode : std::numeric_limits<double>::quiet_NaN();
      hw_joint_states_[i].loop_type = !lite1_ && data.joints[data_idx].control_info_valid
          ? static_cast<uint8_t>(data.joints[data_idx].loop_type)
          : std::numeric_limits<double>::quiet_NaN();
      hw_joint_states_[i].status =
          static_cast<double>(data.joints[data_idx].status);
      hw_joint_states_[i].error =
          static_cast<double>(data.joints[data_idx].error);
    }

    if (data.hand_state_valid) {
      hw_hand_state_[0] = data.hand_state.state;
      hw_hand_state_[1] = data.hand_state.temperature;
      std::copy(data.hand_state.error_warn.begin(), data.hand_state.error_warn.end(),
                hw_hand_state_.begin() + 2);
      std::copy(data.hand_state.error_fault.begin(), data.hand_state.error_fault.end(),
                hw_hand_state_.begin() + 8);
      std::copy(data.hand_state.error_critical.begin(), data.hand_state.error_critical.end(),
                hw_hand_state_.begin() + 20);
    } else {
      hw_hand_state_.fill(std::numeric_limits<double>::quiet_NaN());
    }

    for (size_t i = 0; i < hw_finger_tactile_data_.size(); ++i) {
      if (std::any_of(info_.sensors[i].state_interfaces.begin(),
                      info_.sensors[i].state_interfaces.end(),
                      [](const auto& si) { return si.name == "hand_state"; })) continue;
      if (i < data.tactile_regions.size()) {
        const auto& region = data.tactile_regions[i];
        hw_finger_tactile_data_[i].resultant_force.x =
            static_cast<double>(region.resultant_force.x);
        hw_finger_tactile_data_[i].resultant_force.y =
            static_cast<double>(region.resultant_force.y);
        hw_finger_tactile_data_[i].resultant_force.z =
            static_cast<double>(region.resultant_force.z);

        for (size_t j = 0;
             j < hw_finger_tactile_data_[i].distributed_forces.size(); ++j) {
          if (j < region.distributed_forces.size()) {
            hw_finger_tactile_data_[i].distributed_forces[j].x =
                static_cast<double>(region.distributed_forces[j].x);
            hw_finger_tactile_data_[i].distributed_forces[j].y =
                static_cast<double>(region.distributed_forces[j].y);
            hw_finger_tactile_data_[i].distributed_forces[j].z =
                static_cast<double>(region.distributed_forces[j].z);
          } else {
            hw_finger_tactile_data_[i].distributed_forces[j].x = 0.0;
            hw_finger_tactile_data_[i].distributed_forces[j].y = 0.0;
            hw_finger_tactile_data_[i].distributed_forces[j].z = 0.0;
          }
        }
      } else {
        hw_finger_tactile_data_[i].resultant_force.x = 0.0;
        hw_finger_tactile_data_[i].resultant_force.y = 0.0;
        hw_finger_tactile_data_[i].resultant_force.z = 0.0;
        for (auto& force : hw_finger_tactile_data_[i].distributed_forces) {
          force.x = 0.0;
          force.y = 0.0;
          force.z = 0.0;
        }
      }
    }
  }

  const rclcpp::Duration elapsed = time - last_successful_read_;
  if (elapsed.nanoseconds() > static_cast<int64_t>(read_timeout_ms_ * 1e6)) {
    static rclcpp::Clock throttle_clock(RCL_STEADY_TIME);
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("GHandHardwareInterface"),
                          throttle_clock, 1000,
                          "Hardware read timeout: no valid data for %.1f ms.",
                          read_timeout_ms_);
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GHandHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (!comm_) return hardware_interface::return_type::ERROR;

  std::vector<ghand_comm::JointCommand> commands;
  commands.reserve(command_joint_indices_.size());
  for (size_t idx : command_joint_indices_) {
    const auto& input = hw_joint_commands_[idx];
    if (std::isnan(input.position) && std::isnan(input.velocity) &&
        std::isnan(input.effort) && std::isnan(input.current_percent) &&
        std::isnan(input.control_state)) continue;

    ghand_comm::JointCommand cmd;
    if (!ghand_comm::make_joint_command(input, hand_mode_,
                                        hw_joint_states_[idx].position, &cmd, lite1_))
      return hardware_interface::return_type::ERROR;
    cmd.id = joint_id_map_[idx];
    commands.push_back(cmd);
  }

  if (!comm_->write(commands)) {
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace ghand_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ghand_hardware::GHandHardwareInterface,
                       hardware_interface::SystemInterface)
