// Copyright (c) 2026 GHand ROS2 Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef GHAND_HARDWARE_INTERFACE_COMMUNICATION_MODBUS_CODEC_HPP_
#define GHAND_HARDWARE_INTERFACE_COMMUNICATION_MODBUS_CODEC_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "ghand_hardware_interface/communication/communication.hpp"

namespace ghand_comm {

// GHand5 input layout from the C++ SDK product defaults: a uint16 hand state,
// 6 warning + 12 fault + 6 critical error bytes, and an int16 temperature.
// The joint block starts right after the hand feedback block (0x105F).
constexpr uint16_t kHandFeedbackRegister = 0x1051;
constexpr uint8_t kHandWarningBytes = 6;
constexpr uint8_t kHandFaultBytes = 12;
constexpr uint8_t kHandCriticalBytes = 6;
constexpr size_t kHandFeedbackRegisterCount =
    (2 + kHandWarningBytes + kHandFaultBytes + kHandCriticalBytes + 2) / 2;
constexpr uint16_t kJointInputRegister = static_cast<uint16_t>(
    kHandFeedbackRegister + kHandFeedbackRegisterCount);
constexpr uint16_t kTactileStatusRegister = 0x111A;

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

// Parse the hand feedback block (0x1051~0x105E, 14 registers) into hand state.
HandState parse_hand_info(const uint16_t* raw,
                          size_t count = kHandFeedbackRegisterCount, bool lite1 = false);

// Parse 5 input registers into a single joint state.
JointState parse_joint_data(const uint16_t* raw, JointId joint_id);

// Parse a product-ordered register block into canonical JointId slots.
// Lite1 uses 11 packed records; G5 uses all 18 records (5 registers each).
std::vector<JointState> parse_joints(const uint16_t* raw, size_t count,
                                     const std::vector<JointId>& valid_joints, bool lite1 = false);

// Parse tactile state and error from the first register (kTactileStatusRegister).
// The low byte is the state and the high byte is the error.
std::pair<uint8_t, uint8_t> parse_tactile_state_error(uint16_t first_reg);

// Parse resultant force (Fx, Fy, Fz) for a tactile region.
// raw: first 16 registers starting at kTactileStatusRegister.
// region_index: 0=THUMB, 1=FF, 2=MF, 3=RF, 4=LF.
Force parse_tactile_resultant(const uint16_t* raw, int region_index);

// Parse distributed force data from a big-endian register byte stream.
// Each register carries its low byte first; count is the number of points.
std::vector<Force> parse_tactile_distributed(const uint8_t* data_bytes,
                                             size_t byte_len, int count);

// Hand register 0x0260: enable in the high byte, hand mode in the low byte.
uint16_t encode_hand_mode(HandControlMode mode, bool enabled = true);

std::optional<HandControlMode> parse_hand_control_mode(const std::string& value);

// Convert ROS2 interface values without modifying the controller's command buffers.
bool make_joint_command(const JointCommandInput& input, HandControlMode hand_mode,
                        double fallback_position, JointCommand* command, bool lite1 = false);

// Encode a JointCommand into six holding-register values.
// First word: mode in bits 0-3, loop type in bits 4-7, requested state in bits 8-15.
std::vector<uint16_t> encode_joint_command(const JointCommand& joint);

// Every active Modbus joint is written; omitted joints stop and hold.
std::map<uint16_t, std::vector<uint16_t>> encode_joint_command_blocks(
    const std::vector<JointCommand>& commands,
    const std::map<JointId, uint16_t>& addresses, size_t register_count = 6);

std::vector<uint8_t> encode_ethercat_command(
    HandControlMode mode, const std::vector<JointId>& active_joints,
    const std::map<JointId, JointCommand>& commands, size_t register_count);

}  // namespace ghand_comm

#endif  // GHAND_HARDWARE_INTERFACE_COMMUNICATION_MODBUS_CODEC_HPP_
