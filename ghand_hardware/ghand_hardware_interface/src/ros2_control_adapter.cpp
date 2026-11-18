// Copyright (c) 2026 GHand ROS2 Maintainers
// SPDX-License-Identifier: Apache-2.0
#include "ghand_hardware_interface/ros2_control_adapter.hpp"
#include <algorithm>
#include <cmath>
#include <cctype>
namespace ghand_comm {
std::optional<HandControlMode> parse_hand_control_mode(const std::string& value) {
  std::string name = value;
  std::transform(name.begin(), name.end(), name.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (name == "position" || name == "0") return HandControlMode::POSITION;
  if (name == "speed" || name == "velocity" || name == "1") return HandControlMode::VELOCITY;
  if (name == "current" || name == "2") return HandControlMode::CURRENT;
  if (name == "mit" || name == "3") return HandControlMode::MIT;
  if (name == "custom" || name == "15") return HandControlMode::CUSTOM;
  return std::nullopt;
}

std::optional<TactileSensorId> tactile_sensor_id(std::string name) {
  for (const auto* prefix : {"left_", "right_"}) {
    if (name.rfind(prefix, 0) == 0) name.erase(0, std::string(prefix).size());
  }
  std::transform(name.begin(), name.end(), name.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (name == "thumb") return TactileSensorId::THUMB;
  if (name == "index" || name == "ff") return TactileSensorId::FF;
  if (name == "middle" || name == "mf") return TactileSensorId::MF;
  if (name == "ring" || name == "rf") return TactileSensorId::RF;
  if (name == "little" || name == "lf") return TactileSensorId::LF;
  return std::nullopt;
}

bool make_joint_command(const JointCommandInput& input, HandControlMode hand_mode,
                        double fallback_position, JointCommand* command) {
  if (!command) return false;
  JointCommand cmd;
  constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
  const auto integer_in_range = [](double value, double low, double high) {
    return std::isfinite(value) && value >= low && value <= high &&
           std::trunc(value) == value;
  };
  cmd.control_mode = hand_mode == HandControlMode::CUSTOM
      ? JointControlMode::POSITION : static_cast<JointControlMode>(hand_mode);
  if (!std::isnan(input.control_mode)) {
    if (!integer_in_range(input.control_mode, 0, 3)) return false;
    const auto mode = static_cast<JointControlMode>(input.control_mode);
    if (hand_mode != HandControlMode::CUSTOM && mode != cmd.control_mode)
      return false;
    cmd.control_mode = mode;
  }
  if (!std::isnan(input.loop_type)) {
    if (!integer_in_range(input.loop_type, 0, 1) ||
        (hand_mode != HandControlMode::CUSTOM && input.loop_type != 1))
      return false;
    cmd.loop_type = static_cast<JointLoopType>(input.loop_type);
  }
  if (!std::isnan(input.control_state)) {
    if (!integer_in_range(input.control_state, 0, 2)) return false;
    cmd.control_state = static_cast<JointControlState>(input.control_state);
  }
  const double position = std::isnan(input.position)
      ? (std::isfinite(fallback_position) ? fallback_position : 0.0)
      : input.position;
  const double degrees = position * kRadToDeg;
  if (!std::isfinite(degrees) || degrees < -3276.8 || degrees > 3276.7)
    return false;
  cmd.angle = static_cast<float>(degrees);
  const bool position_mode = cmd.control_mode == JointControlMode::POSITION;
  // Explicit percent limit, or the device default 100% for position mode.
  const double velocity = std::isnan(input.velocity_percent)
      ? (position_mode ? 100.0 : 0.0) : input.velocity_percent;
  const double current = !std::isnan(input.current_percent) ? input.current_percent
      : position_mode ? 100.0
      : cmd.control_mode == JointControlMode::VELOCITY ? 100.0 : 0.0;
  if (!std::isfinite(velocity) || !std::isfinite(current))
    return false;
  cmd.velocity = static_cast<int16_t>(std::clamp(velocity, -100.0, 100.0));
  cmd.current = static_cast<int16_t>(std::clamp(current, position_mode ? 0.0 : -100.0, 100.0));
  if (!std::isnan(input.kp)) {
    if (!integer_in_range(input.kp, -32768, 32767)) return false;
    cmd.kp = static_cast<int16_t>(input.kp);
  }
  if (!std::isnan(input.kd)) {
    if (!integer_in_range(input.kd, -32768, 32767)) return false;
    cmd.kd = static_cast<int16_t>(input.kd);
  }
  *command = cmd;
  return true;
}

}  // namespace ghand_comm
