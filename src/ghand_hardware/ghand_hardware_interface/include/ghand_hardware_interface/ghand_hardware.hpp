// Copyright (c) 2026 GHand ROS2 Maintainers
// SPDX-License-Identifier: MIT

#ifndef GHAND_HARDWARE_INTERFACE_GHAND_HARDWARE_HPP_
#define GHAND_HARDWARE_INTERFACE_GHAND_HARDWARE_HPP_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "ghand_hardware_interface/communication/communication.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace ghand_hardware {

class GHandHardwareInterface : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(GHandHardwareInterface)

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams& params)
      override;

  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_error(
      const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  struct Force {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  struct FingerTactileData {
    Force resultant_force;
    std::vector<Force> distributed_forces;
  };

  struct JointState {
    double position = std::numeric_limits<double>::quiet_NaN();
    double velocity = std::numeric_limits<double>::quiet_NaN();
    double effort = std::numeric_limits<double>::quiet_NaN();
    double status = 0.0;
    double error = 0.0;
  };

  struct JointCommand {
    double position = std::numeric_limits<double>::quiet_NaN();
    double velocity = std::numeric_limits<double>::quiet_NaN();
    double effort = std::numeric_limits<double>::quiet_NaN();
  };

  std::vector<JointState> hw_joint_states_;
  std::vector<JointCommand> hw_joint_commands_;
  std::vector<FingerTactileData> hw_finger_tactile_data_;

  std::unique_ptr<ghand_comm::CommunicationBase> comm_;
  std::vector<ghand_comm::JointId> joint_id_map_;
  std::vector<size_t> command_joint_indices_;
  static constexpr double kDefaultReadTimeoutMs = 500.0;
  double read_timeout_ms_ = kDefaultReadTimeoutMs;
  rclcpp::Time last_successful_read_{0, 0};
};

}  // namespace ghand_hardware

#endif  // GHAND_HARDWARE_INTERFACE_GHAND_HARDWARE_HPP_
