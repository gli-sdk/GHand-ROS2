// Copyright (c) 2026 GHand ROS2 Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <fcntl.h>
#include <pty.h>
#include <sys/select.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "ghand_hardware_interface/communication/communication.hpp"
#include "ghand_hardware_interface/communication/zqwl_canfd_driver.hpp"
#include "gtest/gtest.h"

namespace {

uint32_t pack_id(uint8_t src, uint8_t dst, uint8_t ack, uint8_t function,
                 uint8_t start = 1, uint8_t end = 1, uint8_t toggle = 0,
                 uint8_t segment = 0) {
  return ((src & 0x3F) << 23) | ((dst & 0x3F) << 17) |
         ((ack & 0x01) << 16) | (function << 8) | ((start & 1) << 7) |
         ((end & 1) << 6) | ((toggle & 1) << 5) | (segment & 0x1F);
}

uint8_t valid_length(size_t length) {
  static constexpr uint8_t lengths[] = {0,  1,  2,  3,  4,  5,  6,  7,
                                         8, 12, 16, 20, 24, 32, 48, 64};
  for (uint8_t candidate : lengths) {
    if (candidate >= length) return candidate;
  }
  return 64;
}

bool write_all(int fd, const std::vector<uint8_t>& bytes) {
  size_t offset = 0;
  while (offset < bytes.size()) {
    const ssize_t count = write(fd, bytes.data() + offset,
                                bytes.size() - offset);
    if (count < 0) {
      if (errno == EINTR || errno == EAGAIN) continue;
      return false;
    }
    if (count == 0) return false;
    offset += static_cast<size_t>(count);
  }
  return true;
}

class FakeZqwlAdapter {
 public:
  explicit FakeZqwlAdapter(int master_fd) : master_fd_(master_fd) {}

  void start() { thread_ = std::thread([this]() { run(); }); }

  void stop() {
    stop_.store(true);
    if (thread_.joinable()) thread_.join();
  }

  std::atomic<bool> config_ok{false};
  std::atomic<bool> control_ok{false};
  std::atomic<bool> framing_ok{true};
  std::atomic<bool> established{false};
  std::atomic<bool> read_seen{false};
  std::atomic<bool> mode_write_seen{false};
  std::atomic<bool> joint_write_seen{false};

 private:
  bool read_byte(uint8_t* value) {
    fd_set set;
    FD_ZERO(&set);
    FD_SET(master_fd_, &set);
    timeval timeout{0, 20000};
    const int ready = select(master_fd_ + 1, &set, nullptr, nullptr, &timeout);
    if (ready <= 0) return false;
    return read(master_fd_, value, 1) == 1;
  }

  bool read_exact(std::vector<uint8_t>* bytes, size_t count) {
    bytes->clear();
    bytes->reserve(count);
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (bytes->size() < count && std::chrono::steady_clock::now() < deadline) {
      uint8_t value = 0;
      if (read_byte(&value)) bytes->push_back(value);
    }
    return bytes->size() == count;
  }

  void send_response(uint32_t request_id, const std::vector<uint8_t>& data) {
    const uint8_t request_src = (request_id >> 23) & 0x3F;
    const uint8_t request_dst = (request_id >> 17) & 0x3F;
    const uint8_t function = (request_id >> 8) & 0xFF;
    constexpr size_t kSegmentSize = 64;
    const size_t segment_count =
        std::max<size_t>(1, (data.size() + kSegmentSize - 1) / kSegmentSize);
    for (size_t segment = 0; segment < segment_count; ++segment) {
      const size_t offset = segment * kSegmentSize;
      const size_t chunk =
          std::min(kSegmentSize, data.size() - std::min(offset, data.size()));
      const uint32_t response_id =
          pack_id(request_dst, request_src, 1, function, segment == 0,
                  segment + 1 == segment_count, segment % 2, segment) |
          0x80000000u;
      const uint8_t length = valid_length(chunk);

      std::vector<uint8_t> raw = {
          0x5A, length, 0x05,
          static_cast<uint8_t>((response_id >> 24) & 0xFF),
          static_cast<uint8_t>((response_id >> 16) & 0xFF),
          static_cast<uint8_t>((response_id >> 8) & 0xFF),
          static_cast<uint8_t>(response_id & 0xFF)};
      if (chunk > 0) {
        raw.insert(raw.end(), data.begin() + offset,
                   data.begin() + offset + chunk);
      }
      raw.resize(7 + length, 0);
      raw.push_back(0xA5);
      if (!write_all(master_fd_, raw)) framing_ok.store(false);
    }
  }

  void handle_can_frame(const std::vector<uint8_t>& frame) {
    if (frame.size() < 8 || frame[2] != 0x05 ||
        (frame[3] & 0x80) == 0 || frame.back() != 0xA5) {
      framing_ok.store(false);
      return;
    }
    const uint32_t id =
        ((static_cast<uint32_t>(frame[3]) & 0x7F) << 24) |
        (static_cast<uint32_t>(frame[4]) << 16) |
        (static_cast<uint32_t>(frame[5]) << 8) | frame[6];
    const uint8_t function = (id >> 8) & 0xFF;
    const uint8_t* payload = frame.data() + 7;

    if (function == 0x02) {
      established.store(true);
      send_response(id, {0x00});
      return;
    }
    if (function == 0x04) {
      read_seen.store(true);
      const uint16_t address = (payload[0] << 8) | payload[1];
      const uint16_t count = (payload[2] << 8) | payload[3];
      std::vector<uint8_t> response(1 + count * 2, 0);
      response[0] = static_cast<uint8_t>(count * 2);
      if (address == 0x1023 && count >= 6) {
        // THUMB_IP is joint ID 0; angle is register offset 1.
        response[1 + 1 * 2] = 0x00;
        response[1 + 1 * 2 + 1] = 123;  // 12.3 degrees
        response[1 + 2 * 2] = 20;
      }
      send_response(id, response);
      return;
    }
    if (function == 0x06) {
      const uint16_t address = (payload[0] << 8) | payload[1];
      if (address == 0x0010) mode_write_seen.store(true);
      send_response(id, {payload[0], payload[1], payload[2], payload[3]});
      return;
    }
    if (function == 0x10) {
      const uint16_t address = (payload[0] << 8) | payload[1];
      if (address == 0x0011) joint_write_seen.store(true);
      send_response(id, {payload[0], payload[1], payload[2], payload[3]});
    }
  }

  void run() {
    int config_count = 0;
    while (!stop_.load()) {
      uint8_t head = 0;
      if (!read_byte(&head)) continue;
      if (head == 0x49) {
        std::vector<uint8_t> rest;
        if (!read_exact(&rest, 21)) continue;
        if (rest[0] != 0x3B || rest[19] != 0x45 || rest[20] != 0x2E) {
          framing_ok.store(false);
          continue;
        }
        if (rest[1] == 0x42) {
          config_ok.store(rest[3] == 0 && rest[4] == 0 && rest[5] == 0);
          ++config_count;
        } else if (rest[1] == 0x44) {
          control_ok.store(rest[3] == 1 && rest[5] == 1);
          ++config_count;
        }
        (void)config_count;
      } else if (head == 0x5A) {
        uint8_t length = 0;
        if (!read_byte(&length)) continue;
        std::vector<uint8_t> rest;
        if (!read_exact(&rest, 6 + (length & 0x7F))) continue;
        std::vector<uint8_t> frame = {head, length};
        frame.insert(frame.end(), rest.begin(), rest.end());
        handle_can_frame(frame);
      }
    }
  }

  int master_fd_;
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

TEST(ZqwlCanfdDriver, MatchesGiteeSdkWireProtocol) {
  int master_fd = -1;
  int slave_fd = -1;
  char slave_name[128] = {};
  ASSERT_EQ(openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr), 0);
  ASSERT_GE(master_fd, 0);
  ASSERT_GE(slave_fd, 0);
  close(slave_fd);

  const int flags = fcntl(master_fd, F_GETFL, 0);
  ASSERT_GE(flags, 0);
  ASSERT_EQ(fcntl(master_fd, F_SETFL, flags | O_NONBLOCK), 0);

  FakeZqwlAdapter adapter(master_fd);
  adapter.start();

  ghand_comm::ZqwlCanfdDriver driver;
  const std::unordered_map<std::string, std::string> parameters = {
      {"device", std::string(slave_name) + ":0"},
      {"serial_baud_rate", "2000000"},
      {"bitrate", "1000000"},
      {"data_rate", "5000000"},
      {"node_id", "49"}};

  ASSERT_TRUE(driver.init(parameters));
  EXPECT_TRUE(adapter.config_ok.load());
  EXPECT_TRUE(adapter.control_ok.load());
  EXPECT_TRUE(adapter.established.load());
  EXPECT_TRUE(adapter.framing_ok.load());

  ghand_comm::CommunicationData state;
  const auto state_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < state_deadline) {
    if (driver.read(state) && !state.joints.empty() &&
        state.joints[0].position > 12.2f) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_FALSE(state.joints.empty());
  EXPECT_NEAR(state.joints[0].position, 12.3f, 0.01f);
  EXPECT_TRUE(adapter.read_seen.load());

  ghand_comm::JointCommand command;
  command.id = ghand_comm::JointId::THUMB_MCP;
  command.position = 25.0f;
  command.velocity = 30;
  command.current = 40;
  EXPECT_TRUE(driver.write({command}));
  EXPECT_TRUE(adapter.mode_write_seen.load());
  EXPECT_TRUE(adapter.joint_write_seen.load());

  driver.close();
  adapter.stop();
  close(master_fd);
  EXPECT_TRUE(adapter.framing_ok.load());
}

}  // namespace
