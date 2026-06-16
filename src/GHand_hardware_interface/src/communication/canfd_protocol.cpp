#include "communication/canfd_protocol.hpp"

#include <cstring>

namespace ghand_comm
{
namespace canfd
{

bool PacketAssembler::feed(const Frame & frame, std::vector<uint8_t> * out_payload)
{
  if (out_payload == nullptr) return false;

  ArbitrationId arb(frame.id);
  const uint8_t seq = arb.seq();
  const uint8_t total = arb.total();

  const auto now = std::chrono::steady_clock::now();

  // Timeout detection: if assembly is in progress and not completed within
  // 500 ms, auto-reset.
  if (expected_total_ != 0 && (now - first_frame_time_) > kAssemblyTimeout)
  {
    reset();
  }

  // Single-frame packet (compatible with total=0 or total=1).
  if (total <= 1)
  {
    out_payload->assign(frame.data, frame.data + frame.len);
    return true;
  }

  if (expected_total_ == 0)
  {
    expected_total_ = total;
    frame_offsets_.assign(total, 0);
    frame_lens_.assign(total, 0);
    buffer_.clear();
    received_mask_ = 0;
    first_frame_time_ = now;
  }

  if (total != expected_total_)
  {
    reset();
    expected_total_ = total;
    frame_offsets_.assign(total, 0);
    frame_lens_.assign(total, 0);
    buffer_.clear();
    first_frame_time_ = now;
  }

  if (seq >= total) return false;

  if (received_mask_ & (1 << seq)) return false;  // Duplicate frame

  frame_offsets_[seq] = static_cast<uint16_t>(buffer_.size());
  frame_lens_[seq] = frame.len;
  buffer_.insert(buffer_.end(), frame.data, frame.data + frame.len);
  received_mask_ |= (1 << seq);

  const uint16_t expected_mask = static_cast<uint16_t>((1U << total) - 1);
  if (received_mask_ == expected_mask)
  {
    out_payload->clear();
    for (uint8_t i = 0; i < total; ++i)
    {
      const uint16_t off = frame_offsets_[i];
      const uint8_t len = frame_lens_[i];
      out_payload->insert(
        out_payload->end(), buffer_.begin() + off, buffer_.begin() + off + len);
    }
    reset();
    return true;
  }

  return false;
}

void PacketAssembler::reset()
{
  buffer_.clear();
  frame_offsets_.clear();
  frame_lens_.clear();
  expected_total_ = 0;
  received_mask_ = 0;
  first_frame_time_ = std::chrono::steady_clock::time_point{};
}

uint8_t nearest_valid_canfd_length(uint8_t len)
{
  static constexpr uint8_t kLengths[] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
  for (uint8_t valid : kLengths)
  {
    if (valid >= len) return valid;
  }
  return 64;
}

uint32_t pack_modbus_arbitration_id(
  uint8_t src_id, uint8_t dst_id, uint8_t ack, uint8_t func_code, uint8_t start,
  uint8_t end, uint8_t toggle, uint8_t seg_num)
{
  uint32_t id = 0;
  id |= (static_cast<uint32_t>(src_id) & 0x3F) << 23;
  id |= (static_cast<uint32_t>(dst_id) & 0x3F) << 17;
  id |= (static_cast<uint32_t>(ack) & 0x01) << 16;
  id |= (static_cast<uint32_t>(func_code) & 0xFF) << 8;
  id |= (static_cast<uint32_t>(start) & 0x01) << 7;
  id |= (static_cast<uint32_t>(end) & 0x01) << 6;
  id |= (static_cast<uint32_t>(toggle) & 0x01) << 5;
  id |= (static_cast<uint32_t>(seg_num) & 0x1F);
  return id;
}

std::vector<uint8_t> encode_write_single_register(uint16_t addr, uint16_t value)
{
  return {
    static_cast<uint8_t>((addr >> 8) & 0xFF),
    static_cast<uint8_t>(addr & 0xFF),
    static_cast<uint8_t>((value >> 8) & 0xFF),
    static_cast<uint8_t>(value & 0xFF)};
}

std::vector<uint8_t> encode_write_multiple_registers(
  uint16_t addr, const std::vector<uint8_t> & data)
{
  std::vector<uint8_t> payload;
  payload.reserve(5 + data.size());
  payload.push_back(static_cast<uint8_t>((addr >> 8) & 0xFF));
  payload.push_back(static_cast<uint8_t>(addr & 0xFF));
  const uint16_t reg_count = static_cast<uint16_t>(data.size() / 2);
  payload.push_back(static_cast<uint8_t>((reg_count >> 8) & 0xFF));
  payload.push_back(static_cast<uint8_t>(reg_count & 0xFF));
  payload.push_back(static_cast<uint8_t>(data.size()));
  payload.insert(payload.end(), data.begin(), data.end());
  return payload;
}

}  // namespace canfd
}  // namespace ghand_comm
