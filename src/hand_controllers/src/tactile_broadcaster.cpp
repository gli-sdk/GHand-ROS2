#include "hand_controllers/tactile_broadcaster.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace hand_controllers
{

controller_interface::CallbackReturn TactileBroadcaster::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration TactileBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration TactileBroadcaster::state_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn TactileBroadcaster::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  publisher_ = get_node()->create_publisher<hand_msgs::msg::TactileData>("/tactile_data", 10);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TactileBroadcaster::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TactileBroadcaster::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type TactileBroadcaster::update(const rclcpp::Time & /*time", const rclcpp::Duration & /*period*/)
{
  auto msg = hand_msgs::msg::TactileData();
  // TODO: Fill tactile data from hardware
  publisher_->publish(msg);
  return controller_interface::return_type::OK;
}

}  // namespace hand_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(hand_controllers::TactileBroadcaster, controller_interface::ControllerInterface)
