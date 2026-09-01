// Copyright (c) 2026 GHand ROS2 Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef GHAND_CONTROLLERS_TACTILE_BROADCASTER_HPP_
#define GHAND_CONTROLLERS_TACTILE_BROADCASTER_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "ghand_msgs/msg/tactile_data.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace ghand_controllers {

struct FingerTactileState {
  std::string name;
  hardware_interface::LoanedStateInterface* resultant_force_x = nullptr;
  hardware_interface::LoanedStateInterface* resultant_force_y = nullptr;
  hardware_interface::LoanedStateInterface* resultant_force_z = nullptr;
  std::vector<hardware_interface::LoanedStateInterface*> distributed_force_x;
  std::vector<hardware_interface::LoanedStateInterface*> distributed_force_y;
  std::vector<hardware_interface::LoanedStateInterface*> distributed_force_z;
};

class TactileBroadcaster : public controller_interface::ControllerInterface {
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
  std::shared_ptr<rclcpp::Publisher<ghand_msgs::msg::TactileData>> publisher_;
  std::vector<std::string> finger_names_;
  std::vector<int64_t> point_counts_;
  std::vector<FingerTactileState> finger_states_;
  double publish_rate_ = 100.0;
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};

  void cache_state_interface_pointers();
};

}  // namespace ghand_controllers

#endif  // GHAND_CONTROLLERS_TACTILE_BROADCASTER_HPP_
