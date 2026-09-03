// Copyright (c) 2026 GHand ROS2 Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "ghand_hardware_interface/communication/modbus_codec.hpp"

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
  if (count >= 1) {
    hand_state.state = static_cast<uint8_t>(raw[0] & 0xFF);
  }
  const auto copy_low_high = [raw](size_t start,
                                   std::array<uint8_t, 6>* out) {
    for (size_t i = 0; i < out->size(); ++i) {
      const uint16_t reg = raw[start + i / 2];
      (*out)[i] = static_cast<uint8_t>((i % 2 == 0) ? reg : (reg >> 8));
    }
  };
  if (count >= 4) {
    copy_low_high(1, &hand_state.error_warn);
  }
  if (count >= 7) {
    copy_low_high(4, &hand_state.error_fault);
  }
  if (count >= 10) {
    copy_low_high(7, &hand_state.error_critical);
  }
  if (count >= 11) {
    hand_state.temperature = static_cast<int16_t>(raw[10]);
  }
  return hand_state;
}

JointState parse_joint_data(const uint16_t* raw, JointId joint_id) {
  JointState joint;
  joint.status = static_cast<uint8_t>(raw[0] & 0xFF);
  joint.error = static_cast<uint8_t>((raw[0] >> 8) & 0xFF);
  joint.mode = static_cast<uint8_t>(raw[1] & 0xFF);
  joint.stop = static_cast<uint8_t>((raw[1] >> 8) & 0xFF);

  const int16_t angle_raw = static_cast<int16_t>(raw[2]);
  joint.position = angle_raw / 100.0f;

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
  return {static_cast<uint8_t>((first_reg >> 8) & 0xFF),
          static_cast<uint8_t>(first_reg & 0xFF)};
}

Force parse_tactile_resultant(const uint16_t* raw, int region_index) {
  const int base = 1 + region_index * 3;
  const uint8_t raw_fx = (raw[base] >> 8) & 0xFF;
  const uint8_t raw_fy = (raw[base + 1] >> 8) & 0xFF;
  const uint8_t raw_fz = (raw[base + 2] >> 8) & 0xFF;

  Force f;
  f.x = (raw_fx >= 0x80) ? (raw_fx - 0x100) / 10.0f : raw_fx / 10.0f;
  f.y = (raw_fy >= 0x80) ? (raw_fy - 0x100) / 10.0f : raw_fy / 10.0f;
  f.z = raw_fz / 10.0f;
  return f;
}

std::vector<Force> parse_tactile_distributed(const uint8_t* data_bytes,
                                             size_t byte_len, int count) {
  std::vector<Force> forces;
  forces.reserve(count);

  const int expected_bytes = count * 3;
  const size_t parse_len = (byte_len < static_cast<size_t>(expected_bytes))
                               ? byte_len
                               : static_cast<size_t>(expected_bytes);

  for (size_t i = 0; i + 2 < parse_len; i += 3) {
    Force f;
    f.x = static_cast<float>(static_cast<int8_t>(data_bytes[i])) / 10.0f;
    f.y = static_cast<float>(static_cast<int8_t>(data_bytes[i + 1])) / 10.0f;
    f.z = static_cast<float>(data_bytes[i + 2]) / 10.0f;
    forces.push_back(f);
  }
  return forces;
}

std::vector<uint16_t> encode_joint_command(const JointCommand& joint,
                                           CtrlMode mode) {
  const int16_t angle_raw =
      static_cast<int16_t>(std::lround(joint.position * 10.0f));
  const CtrlMode joint_mode =
      joint.mode == CtrlMode::POSITION ? mode : joint.mode;
  const uint16_t mode_stop =
      static_cast<uint16_t>(((joint.stop & 0xFFu) << 8) |
                            (static_cast<uint8_t>(joint_mode) & 0x0Fu));
  return {
      mode_stop,
      static_cast<uint16_t>(angle_raw),
      static_cast<uint16_t>(static_cast<int16_t>(joint.velocity)),
      static_cast<uint16_t>(static_cast<int16_t>(joint.current)),
      static_cast<uint16_t>(joint.kp),
      static_cast<uint16_t>(joint.kd),
  };
}

}  // namespace ghand_comm
