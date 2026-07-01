// Copyright (c) 2026 GHand ROS2 Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef GHAND_HARDWARE_INTERFACE_COMMUNICATION_CANFD_DRIVER_HPP_
#define GHAND_HARDWARE_INTERFACE_COMMUNICATION_CANFD_DRIVER_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ghand_hardware_interface/communication/communication.hpp"

namespace ghand_comm {

class CanfdDriver : public CommunicationBase {
 public:
  CanfdDriver();
  ~CanfdDriver() override;

  CanfdDriver(const CanfdDriver&) = delete;
  CanfdDriver& operator=(const CanfdDriver&) = delete;

  bool init(
      const std::unordered_map<std::string, std::string>& params) override;
  bool read(CommunicationData& out_data) override;
  bool write(const std::vector<JointCommand>& commands) override;
  void close() override;
  const char* get_name() const override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ghand_comm

#endif  // GHAND_HARDWARE_INTERFACE_COMMUNICATION_CANFD_DRIVER_HPP_
