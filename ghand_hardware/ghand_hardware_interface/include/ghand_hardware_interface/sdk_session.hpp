// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <ghand/ghand.h>
#include <mutex>
#include "ghand_hardware_interface/communication/communication.hpp"

namespace ghand_comm {
class SdkSession final : public CommunicationBase {
 public:
  explicit SdkSession(std::string transport) : transport_(std::move(transport)) {}
  ~SdkSession() override { close(); }
  bool init(const std::unordered_map<std::string, std::string>& params) override;
  bool read(CommunicationData& data) override;
  bool write(const std::vector<JointCommand>& commands) override;
  bool stop() override;
  void close() override;
  const char* get_name() const override { return transport_.c_str(); }

 private:
  struct Feedback {
    std::mutex mutex;
    CommunicationData data;
    std::chrono::steady_clock::time_point hand_received{};
    std::chrono::steady_clock::time_point tactile_received{};
  };
  std::string transport_;
  std::unique_ptr<ghand::Master> master_;
  ghand::GHand* hand_ = nullptr;  // Owned by master_; cleared before disconnect.
  std::shared_ptr<Feedback> feedback_;
  HandControlMode mode_ = HandControlMode::POSITION;
};
}  // namespace ghand_comm
