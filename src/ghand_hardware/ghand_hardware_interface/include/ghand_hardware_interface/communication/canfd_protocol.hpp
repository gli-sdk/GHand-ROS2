// Copyright (c) 2026 GHand ROS2 Maintainers
// SPDX-License-Identifier: MIT

#ifndef GHAND_HARDWARE_INTERFACE_COMMUNICATION_CANFD_PROTOCOL_HPP_
#define GHAND_HARDWARE_INTERFACE_COMMUNICATION_CANFD_PROTOCOL_HPP_

#include <chrono>
#include <cstdint>
#include <vector>

namespace ghand_comm {
namespace canfd {

// Frame type (arbitration field bits [28:25])
enum class FrameType : uint8_t {
  BROADCAST = 0x0,
  COMMAND = 0x1,
  RESPONSE = 0x2,
  ACTIVE_REPORT = 0x3,
};

// 29-bit arbitration field encoding/decoding
struct ArbitrationId {
  uint32_t raw = 0;

  ArbitrationId() = default;
  explicit ArbitrationId(uint32_t id) : raw(id) {}

  /**
   * @brief Construct a 29-bit extended frame arbitration field (CAN ID)
   *
   * Bit field layout (from MSB to LSB):
   *   bit[28:25]  FrameType  (4bit)
   *   bit[24:17]  device_id  (8bit)  Target slave address
   *   bit[16:13]  seq        (4bit)  Frame index in multi-frame assembly
   *   bit[12:9 ]  total      (4bit)  Total number of fragments
   *   bit[8:0  ]             (9bit)  Reserved/unused
   */
  ArbitrationId(FrameType ft, uint8_t dev_id, uint8_t seq, uint8_t total) {
    raw = 0;
    raw |= (static_cast<uint32_t>(ft) & 0xF) << 25;
    raw |= (static_cast<uint32_t>(dev_id) & 0xFF) << 17;
    raw |= (static_cast<uint32_t>(seq) & 0xF) << 13;
    raw |= (static_cast<uint32_t>(total) & 0xF) << 9;
  }

  FrameType frame_type() const {
    return static_cast<FrameType>((raw >> 25) & 0xF);
  }
  uint8_t device_id() const { return static_cast<uint8_t>((raw >> 17) & 0xFF); }
  uint8_t seq() const { return static_cast<uint8_t>((raw >> 13) & 0xF); }
  uint8_t total() const { return static_cast<uint8_t>((raw >> 9) & 0xF); }
};

// CANFD single frame data
struct Frame {
  // 29-bit extended frame arbitration field (CAN ID), packed with frame
  // type/device ID/sequence by ArbitrationId.
  uint32_t id = 0;
  uint8_t data[64] = {0};  // Data field, CANFD max 64 bytes
  // Actual data length, must be one of the CANFD valid lengths:
  // 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64.
  uint8_t len = 0;
  bool is_fd = true;  // true=CANFD format, false=standard CAN format
  bool is_extended = true;
};

// Multi-frame packet assembler
class PacketAssembler {
 public:
  bool feed(const Frame& frame, std::vector<uint8_t>* out_payload);
  void reset();

 private:
  std::vector<uint8_t> buffer_;
  std::vector<uint16_t> frame_offsets_;
  std::vector<uint8_t> frame_lens_;
  uint8_t expected_total_ = 0;
  uint16_t received_mask_ = 0;
  std::chrono::steady_clock::time_point first_frame_time_;
  static constexpr auto kAssemblyTimeout = std::chrono::milliseconds(500);
};

// Modbus-over-CANFD function codes used for command writes.
namespace modbus {
constexpr uint8_t kWriteSingleRegister = 0x06;
constexpr uint8_t kWriteMultipleRegisters = 0x10;
}  // namespace modbus

// Round @p len up to the nearest valid CANFD data length.
// Valid lengths: 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64.
uint8_t nearest_valid_canfd_length(uint8_t len);

// Convert a CANFD DLC code (0~8,9~15) to the actual data byte length.
uint8_t dlc_to_canfd_length(uint8_t dlc);

// Convert a CANFD data byte length to the smallest valid DLC code.
uint8_t canfd_length_to_dlc(uint8_t len);

// Pack a 29-bit extended CAN ID for Modbus-over-CANFD command/response frames.
// Layout (MSB to LSB):
//   bit[28:23] src_id  (6 bits)
//   bit[22:17] dst_id  (6 bits)
//   bit[16]    ack     (1 bit)
//   bit[15:8]  func_code (8 bits)
//   bit[7]     start   (1 bit)
//   bit[6]     end     (1 bit)
//   bit[5]     toggle  (1 bit)
//   bit[4:0]   seg_num (5 bits)
uint32_t pack_modbus_arbitration_id(uint8_t src_id, uint8_t dst_id, uint8_t ack,
                                    uint8_t func_code, uint8_t start,
                                    uint8_t end, uint8_t toggle,
                                    uint8_t seg_num);

// Unpack a 29-bit extended CAN ID used by Modbus-over-CANFD frames.
struct ModbusArbitrationId {
  uint32_t raw = 0;

  explicit ModbusArbitrationId(uint32_t id) : raw(id) {}

  uint8_t src_id() const { return static_cast<uint8_t>((raw >> 23) & 0x3F); }
  uint8_t dst_id() const { return static_cast<uint8_t>((raw >> 17) & 0x3F); }
  uint8_t ack() const { return static_cast<uint8_t>((raw >> 16) & 0x01); }
  uint8_t func_code() const { return static_cast<uint8_t>((raw >> 8) & 0xFF); }
  uint8_t start() const { return static_cast<uint8_t>((raw >> 7) & 0x01); }
  uint8_t end() const { return static_cast<uint8_t>((raw >> 6) & 0x01); }
  uint8_t toggle() const { return static_cast<uint8_t>((raw >> 5) & 0x01); }
  uint8_t seg_num() const { return static_cast<uint8_t>(raw & 0x1F); }
};

// Encode a Modbus FC 0x06 "write single register" payload.
std::vector<uint8_t> encode_write_single_register(uint16_t addr,
                                                  uint16_t value);

// Encode a Modbus FC 0x10 "write multiple registers" payload.
// @p data must contain an even number of big-endian register bytes.
std::vector<uint8_t> encode_write_multiple_registers(
    uint16_t addr, const std::vector<uint8_t>& data);

}  // namespace canfd
}  // namespace ghand_comm

#endif  // GHAND_HARDWARE_INTERFACE_COMMUNICATION_CANFD_PROTOCOL_HPP_
