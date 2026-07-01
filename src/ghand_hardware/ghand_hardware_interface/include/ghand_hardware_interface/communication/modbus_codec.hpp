// Copyright (c) 2026 GHand ROS2 Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef GHAND_HARDWARE_INTERFACE_COMMUNICATION_MODBUS_CODEC_HPP_
#define GHAND_HARDWARE_INTERFACE_COMMUNICATION_MODBUS_CODEC_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "ghand_hardware_interface/communication/communication.hpp"

namespace ghand_comm {

// Convert a list of big-endian uint16 registers to bytes.
std::vector<uint8_t> registers_to_bytes(const std::vector<uint16_t>& registers);

// Parse device name from 16 bytes (8 registers).
std::string parse_device_name(const uint8_t* raw_bytes, size_t len = 16);

// Parse hardware version from 16 bytes (8 registers).
std::string parse_hardware_version(const uint8_t* raw_bytes, size_t len = 16);

// Parse firmware version from 16 bytes (8 registers).
std::string parse_firmware_version(const uint8_t* raw_bytes, size_t len = 16);

// Parse serial number from 16 bytes (8 registers).
uint32_t parse_serial_number(const uint8_t* raw_bytes, size_t len = 16);

// Parse hand type from 2 bytes (1 register).
// Returns 0 for unknown, 1 for left hand, 2 for right hand.
uint8_t parse_hand_type(const uint8_t* raw_bytes, size_t len = 2);

// Parse 2 input registers (0x1021~0x1022) into hand state.
// Returns {state_byte, error_byte, temperature}.
std::tuple<uint8_t, uint8_t, int16_t> parse_hand_info(const uint16_t* raw,
                                                      size_t count = 2);

// Parse 3 input registers into a single joint state.
JointState parse_joint_data(const uint16_t* raw, JointId joint_id);

// Parse joint data for all valid joints from a contiguous register block.
// The block is assumed to start at register 0x1023 and contain
// (max_joint_id + 1) * 3 registers.
std::vector<JointState> parse_joints(const uint16_t* raw, size_t count,
                                     const std::vector<JointId>& valid_joints);

// Parse tactile state and error from the first register (0x1080).
std::pair<uint8_t, uint8_t> parse_tactile_state_error(uint16_t first_reg);

// Parse resultant force (Fx, Fy, Fz) for a tactile region.
// raw: first 16 registers starting at 0x1080.
// region_index: 0=THUMB, 1=FF, 2=MF, 3=RF, 4=LF.
Force parse_tactile_resultant(const uint16_t* raw, int region_index);

// Parse distributed force data from raw bytes.
// count: Number of tactile points for this region.
std::vector<Force> parse_tactile_distributed(const uint8_t* data_bytes,
                                             size_t byte_len, int count);

// Encode a JointCommand into two holding-register values.
// Returns (reg0, reg1).
std::pair<uint16_t, uint16_t> encode_joint_command(const JointCommand& joint);

}  // namespace ghand_comm

#endif  // GHAND_HARDWARE_INTERFACE_COMMUNICATION_MODBUS_CODEC_HPP_
