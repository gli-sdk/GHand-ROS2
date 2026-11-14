// Copyright (c) 2026 GHand ROS2 Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "ghand_hardware_interface/communication/modbus_codec.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace ghand_comm {

std::vector<uint8_t> registers_to_bytes(
    const std::vector<uint16_t>& registers) {
  std::vector<uint8_t> bytes;
  bytes.reserve(registers.size() * 2);
  for (uint16_t reg : registers) {
    bytes.push_back(static_cast<uint8_t>((reg >> 8) & 0xFF));
    bytes.push_back(static_cast<uint8_t>(reg & 0xFF));
  }
  return bytes;
}

static std::string parse_utf8_string(const uint8_t* raw_bytes, size_t len) {
  std::string str(reinterpret_cast<const char*>(raw_bytes), len);
  const size_t end = str.find('\0');
  if (end != std::string::npos) {
    str.resize(end);
  }
  return str;
}

std::string parse_device_name(const uint8_t* raw_bytes, size_t len) {
  return parse_utf8_string(raw_bytes, len);
}

std::string parse_hardware_version(const uint8_t* raw_bytes, size_t len) {
  return parse_utf8_string(raw_bytes, len);
}

std::string parse_firmware_version(const uint8_t* raw_bytes, size_t len) {
  return parse_utf8_string(raw_bytes, len);
}

uint32_t parse_serial_number(const uint8_t* raw_bytes, size_t len) {
  uint32_t serial = 0;
  for (size_t i = 0; i < len && i < 4; ++i) {
    serial = (serial << 8) | raw_bytes[i];
  }
  return serial;
}

uint8_t parse_hand_type(const uint8_t* raw_bytes, size_t len) {
  if (len < 2) return 0;
  return raw_bytes[1];
}

HandState parse_hand_info(const uint16_t* raw, size_t count) {
  HandState hand_state;
  if (raw == nullptr || count < kHandFeedbackRegisterCount) {
    return hand_state;
  }
  hand_state.state = raw[0];
  size_t offset = 2;  // Bytes consumed by the uint16 state.
  const auto copy_errors = [&](uint8_t* out, size_t bytes) {
    for (size_t i = 0; i < bytes; ++i, ++offset) {
      out[i] = static_cast<uint8_t>(raw[offset / 2] >> ((offset % 2) * 8));
    }
  };
  copy_errors(hand_state.error_warn.data(), kHandWarningBytes);
  copy_errors(hand_state.error_fault.data(), kHandFaultBytes);
  copy_errors(hand_state.error_critical.data(), kHandCriticalBytes);
  hand_state.temperature = static_cast<int16_t>(raw[offset / 2]);
  return hand_state;
}

JointState parse_joint_data(const uint16_t* raw, JointId joint_id) {
  JointState joint;
  joint.status = static_cast<uint8_t>(raw[0] & 0xFF);
  joint.error = static_cast<uint8_t>((raw[0] >> 8) & 0xFF);
  joint.mode = static_cast<uint8_t>(raw[1] & 0x0F);
  joint.loop_type = static_cast<JointLoopType>((raw[1] >> 4) & 0x0F);
  joint.control_info_valid = true;
  joint.stop = static_cast<uint8_t>((raw[1] >> 8) & 0xFF);

  const int16_t angle_raw = static_cast<int16_t>(raw[2]);
  joint.position = angle_raw / 10.0f;

  joint.velocity = static_cast<float>(static_cast<int16_t>(raw[3]));
  joint.current = static_cast<float>(static_cast<int16_t>(raw[4]));

  (void)joint_id;
  return joint;
}

std::vector<JointState> parse_joints(const uint16_t* raw, size_t count,
                                     const std::vector<JointId>& valid_joints) {
  std::vector<JointState> joints;
  joints.reserve(valid_joints.size());

  for (JointId joint_id : valid_joints) {
    const size_t offset = static_cast<size_t>(joint_id) * 5;
    if (offset + 4 >= count) continue;
    joints.push_back(parse_joint_data(raw + offset, joint_id));
  }
  return joints;
}

std::pair<uint8_t, uint8_t> parse_tactile_state_error(uint16_t first_reg) {
  return {static_cast<uint8_t>(first_reg & 0xFF),
          static_cast<uint8_t>((first_reg >> 8) & 0xFF)};
}

Force parse_tactile_resultant(const uint16_t* raw, int region_index) {
  const int base = 1 + region_index * 3;

  Force f;
  f.x = static_cast<float>(static_cast<int8_t>(raw[base] & 0xFF)) / 10.0f;
  f.y = static_cast<float>(static_cast<int8_t>(raw[base + 1] & 0xFF)) / 10.0f;
  f.z = static_cast<float>(raw[base + 2] & 0xFF) / 10.0f;
  return f;
}

std::vector<Force> parse_tactile_distributed(const uint8_t* data_bytes,
                                             size_t byte_len, int count) {
  std::vector<Force> forces;
  forces.reserve(count);
  if (data_bytes == nullptr || count <= 0) return forces;

  const size_t register_bytes = (static_cast<size_t>(count) * 3 + 1) / 2 * 2;
  const size_t parse_len = std::min(byte_len, register_bytes);

  // Registers arrive big-endian; sample bytes are low-byte first.
  std::vector<uint8_t> sample_bytes;
  sample_bytes.reserve(parse_len);
  for (size_t i = 0; i + 1 < parse_len; i += 2) {
    sample_bytes.push_back(data_bytes[i + 1]);
    sample_bytes.push_back(data_bytes[i]);
  }

  for (size_t i = 0; i + 2 < sample_bytes.size(); i += 3) {
    Force f;
    f.x = static_cast<float>(static_cast<int8_t>(sample_bytes[i])) / 10.0f;
    f.y = static_cast<float>(static_cast<int8_t>(sample_bytes[i + 1])) / 10.0f;
    f.z = static_cast<float>(sample_bytes[i + 2]) / 10.0f;
    forces.push_back(f);
  }
  return forces;
}

uint16_t encode_hand_mode(HandControlMode mode, bool enabled) {
  return (static_cast<uint16_t>(enabled) << 8) | static_cast<uint8_t>(mode);
}

std::optional<HandControlMode> parse_hand_control_mode(const std::string& value) {
  std::string name = value;
  std::transform(name.begin(), name.end(), name.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (name == "position" || name == "0") return HandControlMode::POSITION;
  if (name == "speed" || name == "velocity" || name == "1") return HandControlMode::SPEED;
  if (name == "current" || name == "2") return HandControlMode::CURRENT;
  if (name == "mit" || name == "3") return HandControlMode::MIT;
  if (name == "custom" || name == "15") return HandControlMode::CUSTOM;
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
  cmd.mode = hand_mode == HandControlMode::CUSTOM
      ? CtrlMode::POSITION : static_cast<CtrlMode>(hand_mode);
  if (!std::isnan(input.control_mode)) {
    if (!integer_in_range(input.control_mode, 0, 3)) return false;
    const auto mode = static_cast<CtrlMode>(input.control_mode);
    if (hand_mode != HandControlMode::CUSTOM && mode != cmd.mode)
      return false;
    cmd.mode = mode;
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
  cmd.position = static_cast<float>(degrees);
  const bool position_mode = cmd.mode == CtrlMode::POSITION;
  // Position trajectory velocities are rad/s; retain the existing 100% limit.
  const double velocity = position_mode ? 100.0
      : (std::isnan(input.velocity) ? 0.0 : input.velocity);
  const double current = !std::isnan(input.current_percent) ? input.current_percent
      : position_mode ? 100.0
      : !std::isnan(input.effort) ? input.effort
      : cmd.mode == CtrlMode::SPEED ? 100.0 : 0.0;
  if (!std::isfinite(velocity) || !std::isfinite(current))
    return false;
  cmd.velocity = static_cast<int8_t>(std::clamp(velocity, -100.0, 100.0));
  cmd.current = static_cast<int8_t>(std::clamp(current, position_mode ? 0.0 : -100.0, 100.0));
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

std::vector<uint16_t> encode_joint_command(const JointCommand& joint) {
  const int16_t angle_raw =
      static_cast<int16_t>(joint.position * 10.0f);
  const uint16_t mode_stop = static_cast<uint16_t>(
      (static_cast<uint8_t>(joint.control_state) << 8) |
      ((static_cast<uint8_t>(joint.loop_type) & 0x0Fu) << 4) |
      (static_cast<uint8_t>(joint.mode) & 0x0Fu));
  return {
      mode_stop,
      static_cast<uint16_t>(angle_raw),
      static_cast<uint16_t>(static_cast<int16_t>(joint.velocity)),
      static_cast<uint16_t>(static_cast<int16_t>(joint.current)),
      static_cast<uint16_t>(joint.kp),
      static_cast<uint16_t>(joint.kd),
  };
}

std::map<uint16_t, std::vector<uint16_t>> encode_joint_command_blocks(
    const std::vector<JointCommand>& commands,
    const std::map<JointId, uint16_t>& addresses) {
  std::map<JointId, JointCommand> by_id;
  for (const auto& command : commands) by_id[command.id] = command;
  std::map<uint16_t, std::vector<uint16_t>> blocks;
  for (const auto& entry : addresses) {
    JointCommand command;
    command.id = entry.first;
    command.control_state = JointControlState::STOPPED;
    const auto found = by_id.find(entry.first);
    if (found != by_id.end()) command = found->second;
    blocks[entry.second] = encode_joint_command(command);
  }
  return blocks;
}

}  // namespace ghand_comm
