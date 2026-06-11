#include "hand_controllers/joint_status_broadcaster.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace hand_controllers
{

controller_interface::CallbackReturn JointStatusBroadcaster::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration JointStatusBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration JointStatusBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // TODO: Configure which state interfaces to use
  return config;
}

controller_interface::CallbackReturn JointStatusBroadcaster::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  publisher_ = get_node()->create_publisher<hand_msgs::msg::JointStatus>("/joint_status", 10);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointStatusBroadcaster::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointStatusBroadcaster::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type JointStatusBroadcaster::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto msg = hand_msgs::msg::JointStatus();
  // TODO: Fill status from state interfaces
  publisher_->publish(msg);
  return controller_interface::return_type::OK;
}

}  // namespace hand_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(hand_controllers::JointStatusBroadcaster, controller_interface::ControllerInterface)
