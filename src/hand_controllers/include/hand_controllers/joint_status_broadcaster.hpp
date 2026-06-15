#ifndef HAND_CONTROLLERS__JOINT_STATUS_BROADCASTER_HPP_
#define HAND_CONTROLLERS__JOINT_STATUS_BROADCASTER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "hand_msgs/msg/joint_status.hpp"

namespace hand_controllers
{

class JointStatusBroadcaster : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::shared_ptr<rclcpp::Publisher<hand_msgs::msg::JointStatus>> publisher_;
  std::vector<std::string> joint_names_;
  double publish_rate_ = 100.0;
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace hand_controllers

#endif  // HAND_CONTROLLERS__JOINT_STATUS_BROADCASTER_HPP_
