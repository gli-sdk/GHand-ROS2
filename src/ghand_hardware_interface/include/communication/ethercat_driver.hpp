#ifndef GHAND_HARDWARE__ETHERCAT_DRIVER_HPP_
#define GHAND_HARDWARE__ETHERCAT_DRIVER_HPP_

#include <string>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "communication/communication.hpp"

namespace ghand_comm
{

class EthercatDriver : public CommunicationBase
{
public:
  EthercatDriver();
  ~EthercatDriver() override;

  EthercatDriver(const EthercatDriver &) = delete;
  EthercatDriver & operator=(const EthercatDriver &) = delete;

  bool init(const std::unordered_map<std::string, std::string> & params) override;
  bool read(CommunicationData & out_data) override;
  bool write(const std::vector<JointCommand> & commands) override;
  void close() override;
  const char * get_name() const override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ghand_comm

#endif  // GHAND_HARDWARE__ETHERCAT_DRIVER_HPP_
