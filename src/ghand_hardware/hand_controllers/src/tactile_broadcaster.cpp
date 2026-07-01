// Copyright (c) 2026 GHand ROS2 Maintainers
// SPDX-License-Identifier: MIT

#include "hand_controllers/tactile_broadcaster.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace {

double safe_state_value(const hardware_interface::LoanedStateInterface* iface) {
  if (iface == nullptr) {
    return 0.0;
  }
  const double value = iface->get_optional().value_or(0.0);
  return std::isnan(value) ? 0.0 : value;
}

}  // namespace

namespace hand_controllers {

controller_interface::CallbackReturn TactileBroadcaster::on_init() {
  get_node()->declare_parameter<std::vector<std::string>>(
      "fingers", std::vector<std::string>());
  get_node()->declare_parameter<std::vector<int64_t>>("point_counts",
                                                      std::vector<int64_t>());
  get_node()->declare_parameter<double>("publish_rate", 100.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
TactileBroadcaster::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
TactileBroadcaster::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  std::vector<std::string> finger_names;
  std::vector<int64_t> point_counts;
  if (!get_node()->get_parameter("fingers", finger_names) ||
      !get_node()->get_parameter("point_counts", point_counts)) {
    return config;
  }

  for (size_t i = 0; i < finger_names.size(); ++i) {
    const int64_t point_count = (i < point_counts.size()) ? point_counts[i] : 0;
    config.names.push_back(finger_names[i] + "/resultant_force_x");
    config.names.push_back(finger_names[i] + "/resultant_force_y");
    config.names.push_back(finger_names[i] + "/resultant_force_z");
    for (int64_t j = 0; j < point_count; ++j) {
      const std::string base_name =
          finger_names[i] + "/distributed_force_" + std::to_string(j);
      config.names.push_back(base_name + "_x");
      config.names.push_back(base_name + "_y");
      config.names.push_back(base_name + "_z");
    }
  }

  return config;
}

controller_interface::CallbackReturn TactileBroadcaster::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  finger_names_ = get_node()->get_parameter("fingers").as_string_array();
  point_counts_ = get_node()->get_parameter("point_counts").as_integer_array();
  publish_rate_ = get_node()->get_parameter("publish_rate").as_double();

  if (point_counts_.size() != finger_names_.size()) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Parameter 'point_counts' (%zu entries) must match 'fingers' "
                 "(%zu entries).",
                 point_counts_.size(), finger_names_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  publisher_ = get_node()->create_publisher<hand_msgs::msg::TactileData>(
      "/tactile_data", 10);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TactileBroadcaster::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  last_publish_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  cache_state_interface_pointers();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TactileBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type TactileBroadcaster::update(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
  if (publish_rate_ <= 0.0) {
    return controller_interface::return_type::OK;
  }

  const rclcpp::Duration min_interval =
      rclcpp::Duration::from_seconds(1.0 / publish_rate_);
  if (time - last_publish_time_ < min_interval) {
    return controller_interface::return_type::OK;
  }
  last_publish_time_ = time;

  auto msg = hand_msgs::msg::TactileData();
  msg.resultant_force.resize(finger_names_.size() * 3, 0.0);

  size_t distributed_total_size = 0;
  for (const auto& finger : finger_states_) {
    distributed_total_size += finger.distributed_force_x.size() * 3;
  }
  msg.distributed_force.resize(distributed_total_size, 0.0);

  size_t total_offset = 0;
  size_t distributed_offset = 0;
  for (const auto& finger : finger_states_) {
    msg.resultant_force[total_offset + 0] =
        safe_state_value(finger.resultant_force_x);
    msg.resultant_force[total_offset + 1] =
        safe_state_value(finger.resultant_force_y);
    msg.resultant_force[total_offset + 2] =
        safe_state_value(finger.resultant_force_z);
    total_offset += 3;

    for (size_t j = 0; j < finger.distributed_force_x.size(); ++j) {
      msg.distributed_force[distributed_offset + 0] =
          safe_state_value(finger.distributed_force_x[j]);
      msg.distributed_force[distributed_offset + 1] =
          safe_state_value(finger.distributed_force_y[j]);
      msg.distributed_force[distributed_offset + 2] =
          safe_state_value(finger.distributed_force_z[j]);
      distributed_offset += 3;
    }
  }

  publisher_->publish(msg);
  return controller_interface::return_type::OK;
}

void TactileBroadcaster::cache_state_interface_pointers() {
  finger_states_.clear();
  finger_states_.reserve(finger_names_.size());

  for (size_t i = 0; i < finger_names_.size(); ++i) {
    FingerTactileState state;
    state.name = finger_names_[i];
    const int64_t point_count =
        (i < point_counts_.size()) ? point_counts_[i] : 0;
    state.distributed_force_x.resize(point_count, nullptr);
    state.distributed_force_y.resize(point_count, nullptr);
    state.distributed_force_z.resize(point_count, nullptr);

    for (auto& si : state_interfaces_) {
      const std::string name = si.get_name();
      const std::string prefix = state.name + "/";
      if (name.rfind(prefix, 0) != 0) {
        continue;
      }

      const std::string suffix = name.substr(prefix.size());
      if (suffix == "resultant_force_x") {
        state.resultant_force_x = &si;
      } else if (suffix == "resultant_force_y") {
        state.resultant_force_y = &si;
      } else if (suffix == "resultant_force_z") {
        state.resultant_force_z = &si;
      } else if (suffix.rfind("distributed_force_", 0) == 0) {
        const size_t axis_pos = suffix.find_last_of('_');
        if (axis_pos == std::string::npos || axis_pos + 1 >= suffix.size()) {
          continue;
        }
        const std::string index_str = suffix.substr(18, axis_pos - 18);
        if (index_str.empty()) {
          continue;
        }
        const char axis = suffix.back();
        const int64_t index = std::stoll(index_str);
        if (index < 0 || index >= point_count) {
          continue;
        }
        if (axis == 'x') {
          state.distributed_force_x[index] = &si;
        } else if (axis == 'y') {
          state.distributed_force_y[index] = &si;
        } else if (axis == 'z') {
          state.distributed_force_z[index] = &si;
        }
      }
    }

    finger_states_.push_back(state);
  }
}

}  // namespace hand_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(hand_controllers::TactileBroadcaster,
                       controller_interface::ControllerInterface)
