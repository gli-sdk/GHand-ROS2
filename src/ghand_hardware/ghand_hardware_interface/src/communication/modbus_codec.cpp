// Copyright (c) 2026 GHand ROS2 Maintainers
// SPDX-License-Identifier: MIT

#include "ghand_hardware_interface/communication/modbus_codec.hpp"

#include <cmath>
#include <string>
#include <tuple>
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

std::tuple<uint8_t, uint8_t, int16_t> parse_hand_info(const uint16_t* raw,
                                                      size_t count) {
  uint8_t state = 0;
  uint8_t error = 0;
  int16_t temperature = 0;
  if (count >= 1) {
    state = static_cast<uint8_t>((raw[0] >> 8) & 0xFF);
    error = static_cast<uint8_t>(raw[0] & 0xFF);
  }
  if (count >= 2) {
    temperature = static_cast<int16_t>(raw[1]);
  }
  return {state, error, temperature};
}

JointState parse_joint_data(const uint16_t* raw, JointId joint_id) {
  JointState joint;
  joint.status = static_cast<uint8_t>((raw[0] >> 8) & 0xFF);
  joint.error = static_cast<uint8_t>(raw[0] & 0xFF);

  const int16_t angle_raw = static_cast<int16_t>(raw[1]);
  joint.position = angle_raw / 10.0f;

  const uint8_t speed = (raw[2] >> 8) & 0xFF;
  const uint8_t torque = raw[2] & 0xFF;
  joint.velocity = static_cast<float>(static_cast<int8_t>(speed));
  (void)torque;

  (void)joint_id;
  return joint;
}

std::vector<JointState> parse_joints(const uint16_t* raw, size_t count,
                                     const std::vector<JointId>& valid_joints) {
  std::vector<JointState> joints;
  joints.reserve(valid_joints.size());

  for (JointId joint_id : valid_joints) {
    const size_t offset = static_cast<size_t>(joint_id) * 3;
    if (offset + 2 >= count) continue;
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

std::pair<uint16_t, uint16_t> encode_joint_command(const JointCommand& joint) {
  const int16_t angle_raw =
      static_cast<int16_t>(std::lround(joint.position * 10.0f));
  const uint16_t reg0 = static_cast<uint16_t>(angle_raw);
  const uint16_t reg1 = ((static_cast<uint16_t>(joint.velocity) & 0xFF) << 8) |
                        (static_cast<uint16_t>(joint.torque) & 0xFF);
  return {reg0, reg1};
}

}  // namespace ghand_comm
