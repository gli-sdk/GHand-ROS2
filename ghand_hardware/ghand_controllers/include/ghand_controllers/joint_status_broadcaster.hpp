// Copyright (c) 2026 GHand ROS2 Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef GHAND_CONTROLLERS_JOINT_STATUS_BROADCASTER_HPP_
#define GHAND_CONTROLLERS_JOINT_STATUS_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "ghand_msgs/msg/joint_status.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace ghand_controllers {

class JointStatusBroadcaster
    : public controller_interface::ControllerInterface {
 public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;

  controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  std::shared_ptr<rclcpp::Publisher<ghand_msgs::msg::JointStatus>> publisher_;
  std::vector<std::string> joint_names_;
  double publish_rate_ = 100.0;
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace ghand_controllers

#endif  // GHAND_CONTROLLERS_JOINT_STATUS_BROADCASTER_HPP_
