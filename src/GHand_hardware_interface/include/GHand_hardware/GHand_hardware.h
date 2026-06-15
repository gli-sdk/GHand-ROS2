#ifndef GHAND_HARDWARE__GHAND_HARDWARE_H_
#define GHAND_HARDWARE__GHAND_HARDWARE_H_

#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace GHand_hardware
{

class GHandHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(GHandHardwareInterface)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Force
  {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  };

  struct FingerTactileData
  {
    Force resultant_force;
    std::vector<Force> distributed_forces;
  };

  struct JointState
  {
    double position = std::numeric_limits<double>::quiet_NaN();
    double velocity = std::numeric_limits<double>::quiet_NaN();
    double effort = std::numeric_limits<double>::quiet_NaN();
    double status = 0.0;
    double error = 0.0;
  };

  struct JointCommand
  {
    double position = std::numeric_limits<double>::quiet_NaN();
    double velocity = std::numeric_limits<double>::quiet_NaN();
    double effort = std::numeric_limits<double>::quiet_NaN();
  };

  std::vector<JointState> hw_joint_states_;
  std::vector<JointCommand> hw_joint_commands_;
  std::vector<FingerTactileData> hw_finger_tactile_data_;
};

}  // namespace GHand_hardware

#endif  // GHAND_HARDWARE__GHAND_HARDWARE_H_
