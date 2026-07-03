// Copyright (c) 2026 GHand ROS2 Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "hand_controllers/joint_status_broadcaster.hpp"

#include <cmath>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace {

int8_t state_to_int8(double value) {
  if (std::isnan(value)) {
    return 0;
  }
  if (value >= 127.0) {
    return 127;
  }
  if (value <= -128.0) {
    return -128;
  }
  return static_cast<int8_t>(value);
}

}  // namespace

namespace hand_controllers {

controller_interface::CallbackReturn JointStatusBroadcaster::on_init() {
  get_node()->declare_parameter<std::vector<std::string>>(
      "joints", std::vector<std::string>());
  get_node()->declare_parameter<double>("publish_rate", 100.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
JointStatusBroadcaster::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
JointStatusBroadcaster::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  std::vector<std::string> joint_names;
  if (!get_node()->get_parameter("joints", joint_names)) {
    return config;
  }

  for (const auto& joint : joint_names) {
    config.names.push_back(joint + "/state");
    config.names.push_back(joint + "/error");
  }

  return config;
}

controller_interface::CallbackReturn JointStatusBroadcaster::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  publish_rate_ = get_node()->get_parameter("publish_rate").as_double();

  publisher_ = get_node()->create_publisher<hand_msgs::msg::JointStatus>(
      "/joint_status", 10);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointStatusBroadcaster::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  last_publish_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointStatusBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type JointStatusBroadcaster::update(
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

  auto msg = hand_msgs::msg::JointStatus();
  msg.status.resize(joint_names_.size(), 0);
  msg.error.resize(joint_names_.size(), 0);

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const std::string state_name = joint_names_[i] + "/state";
    const std::string error_name = joint_names_[i] + "/error";
    for (const auto& si : state_interfaces_) {
      if (si.get_name() == state_name) {
        msg.status[i] = state_to_int8(si.get_optional().value_or(0.0));
      } else if (si.get_name() == error_name) {
        msg.error[i] = state_to_int8(si.get_optional().value_or(0.0));
      }
    }
  }

  publisher_->publish(msg);
  return controller_interface::return_type::OK;
}

}  // namespace hand_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(hand_controllers::JointStatusBroadcaster,
                       controller_interface::ControllerInterface)
