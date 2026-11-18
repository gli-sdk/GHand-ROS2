// Copyright (c) 2026 GHand ROS2 Maintainers
// SPDX-License-Identifier: Apache-2.0
#ifndef GHAND_HARDWARE_INTERFACE_ROS2_CONTROL_ADAPTER_HPP_
#define GHAND_HARDWARE_INTERFACE_ROS2_CONTROL_ADAPTER_HPP_
#include <limits>
#include <optional>
#include <string>
#include "ghand_hardware_interface/communication/sdk_types.hpp"
namespace ghand_comm {
std::optional<HandControlMode> parse_hand_control_mode(const std::string& value);
std::optional<TactileSensorId> tactile_sensor_id(std::string name);
// ROS position is in radians; custom velocity/current interfaces are percentages.
struct JointCommandInput {
  double position = std::numeric_limits<double>::quiet_NaN();
  double velocity_percent = std::numeric_limits<double>::quiet_NaN();
  double current_percent = std::numeric_limits<double>::quiet_NaN();
  double control_mode = std::numeric_limits<double>::quiet_NaN();
  double loop_type = std::numeric_limits<double>::quiet_NaN();
  double control_state = std::numeric_limits<double>::quiet_NaN();
  double kp = std::numeric_limits<double>::quiet_NaN();
  double kd = std::numeric_limits<double>::quiet_NaN();
};

bool make_joint_command(const JointCommandInput& input, HandControlMode hand_mode,
                        double fallback_position, JointCommand* command);
}  // namespace ghand_comm
#endif
