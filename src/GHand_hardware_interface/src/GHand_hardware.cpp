#include "GHand_hardware/GHand_hardware.h"

#include <vector>
#include <string>
#include <limits>
#include <cmath>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace GHand_hardware
{

hardware_interface::CallbackReturn GHandHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const size_t joint_count = info_.joints.size();
  hw_joint_states_.resize(joint_count);
  hw_joint_commands_.resize(joint_count);
  hw_finger_tactile_data_.resize(info_.sensors.size());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.empty())
    {
      RCLCPP_FATAL(rclcpp::get_logger("GHandHardwareInterface"),
                   "Joint '%s' has no command interfaces.",
                   joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    for (const auto & ci : joint.command_interfaces)
    {
      if (ci.name != hardware_interface::HW_IF_POSITION &&
          ci.name != hardware_interface::HW_IF_VELOCITY &&
          ci.name != hardware_interface::HW_IF_EFFORT)
      {
        RCLCPP_FATAL(rclcpp::get_logger("GHandHardwareInterface"),
                     "Joint '%s' has unsupported command interface '%s'.",
                     joint.name.c_str(), ci.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    if (joint.state_interfaces.empty())
    {
      RCLCPP_FATAL(rclcpp::get_logger("GHandHardwareInterface"),
                   "Joint '%s' has no state interfaces.",
                   joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    bool has_position = false;
    bool has_velocity = false;
    bool has_effort = false;
    for (const auto & si : joint.state_interfaces)
    {
      if (si.name == hardware_interface::HW_IF_POSITION)
      {
        has_position = true;
      }
      else if (si.name == hardware_interface::HW_IF_VELOCITY)
      {
        has_velocity = true;
      }
      else if (si.name == hardware_interface::HW_IF_EFFORT)
      {
        has_effort = true;
      }
    }

    if (!has_position)
    {
      RCLCPP_FATAL(rclcpp::get_logger("GHandHardwareInterface"),
                   "Joint '%s' is missing required state interface '%s'.",
                   joint.name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!has_velocity)
    {
      RCLCPP_FATAL(rclcpp::get_logger("GHandHardwareInterface"),
                   "Joint '%s' is missing required state interface '%s'.",
                   joint.name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!has_effort)
    {
      RCLCPP_FATAL(rclcpp::get_logger("GHandHardwareInterface"),
                   "Joint '%s' is missing required state interface '%s'.",
                   joint.name.c_str(), hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  for (size_t i = 0; i < info_.sensors.size(); ++i)
  {
    const hardware_interface::ComponentInfo & sensor = info_.sensors[i];
    bool has_resultant_force_x = false;
    bool has_resultant_force_y = false;
    bool has_resultant_force_z = false;
    size_t distributed_point_count = 0;

    for (const auto & si : sensor.state_interfaces)
    {
      if (si.name == "resultant_force_x")
      {
        has_resultant_force_x = true;
      }
      else if (si.name == "resultant_force_y")
      {
        has_resultant_force_y = true;
      }
      else if (si.name == "resultant_force_z")
      {
        has_resultant_force_z = true;
      }
      else if (
        si.name.rfind("distributed_force_", 0) == 0 &&
        si.name.size() > 2 && si.name.substr(si.name.size() - 2) == "_x")
      {
        ++distributed_point_count;
      }
    }

    if (!has_resultant_force_x || !has_resultant_force_y || !has_resultant_force_z)
    {
      RCLCPP_FATAL(rclcpp::get_logger("GHandHardwareInterface"),
                   "Sensor '%s' is missing required resultant_force state interfaces.",
                   sensor.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    hw_finger_tactile_data_[i].distributed_forces.resize(distributed_point_count);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GHandHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("GHandHardwareInterface"), "Configuring ...please wait...");

  // TODO: Initialize communication (EtherCAT/RS485/CANFD)

  RCLCPP_INFO(rclcpp::get_logger("GHandHardwareInterface"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> GHandHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    for (const auto & si : info_.joints[i].state_interfaces)
    {
      if (si.name == hardware_interface::HW_IF_POSITION)
      {
        state_interfaces.emplace_back(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION,
            &hw_joint_states_[i].position);
      }
      else if (si.name == hardware_interface::HW_IF_VELOCITY)
      {
        state_interfaces.emplace_back(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
            &hw_joint_states_[i].velocity);
      }
      else if (si.name == hardware_interface::HW_IF_EFFORT)
      {
        state_interfaces.emplace_back(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
            &hw_joint_states_[i].effort);
      }
      else if (si.name == "state")
      {
        state_interfaces.emplace_back(
            info_.joints[i].name, "state", &hw_joint_states_[i].status);
      }
      else if (si.name == "error")
      {
        state_interfaces.emplace_back(
            info_.joints[i].name, "error", &hw_joint_states_[i].error);
      }
    }
  }

  for (size_t i = 0; i < info_.sensors.size(); ++i)
  {
    state_interfaces.emplace_back(
        info_.sensors[i].name, "resultant_force_x", &hw_finger_tactile_data_[i].resultant_force.x);
    state_interfaces.emplace_back(
        info_.sensors[i].name, "resultant_force_y", &hw_finger_tactile_data_[i].resultant_force.y);
    state_interfaces.emplace_back(
        info_.sensors[i].name, "resultant_force_z", &hw_finger_tactile_data_[i].resultant_force.z);

    for (size_t j = 0; j < hw_finger_tactile_data_[i].distributed_forces.size(); ++j)
    {
      const std::string base_name = "distributed_force_" + std::to_string(j);
      state_interfaces.emplace_back(
          info_.sensors[i].name, base_name + "_x",
          &hw_finger_tactile_data_[i].distributed_forces[j].x);
      state_interfaces.emplace_back(
          info_.sensors[i].name, base_name + "_y",
          &hw_finger_tactile_data_[i].distributed_forces[j].y);
      state_interfaces.emplace_back(
          info_.sensors[i].name, base_name + "_z",
          &hw_finger_tactile_data_[i].distributed_forces[j].z);
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> GHandHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    for (const auto & ci : info_.joints[i].command_interfaces)
    {
      if (ci.name == hardware_interface::HW_IF_POSITION)
      {
        command_interfaces.emplace_back(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION,
            &hw_joint_commands_[i].position);
      }
      else if (ci.name == hardware_interface::HW_IF_VELOCITY)
      {
        command_interfaces.emplace_back(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
            &hw_joint_commands_[i].velocity);
      }
      else if (ci.name == hardware_interface::HW_IF_EFFORT)
      {
        command_interfaces.emplace_back(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
            &hw_joint_commands_[i].effort);
      }
    }
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn GHandHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("GHandHardwareInterface"), "Activating ...please wait...");

  for (auto & state : hw_joint_states_)
  {
    if (std::isnan(state.position)) { state.position = 0.0; }
    if (std::isnan(state.velocity)) { state.velocity = 0.0; }
    if (std::isnan(state.effort)) { state.effort = 0.0; }
  }

  // Leave command interfaces as NaN so write() can detect which ones the active
  // controller actually updates and fill the defaults for the current mode.

  RCLCPP_INFO(rclcpp::get_logger("GHandHardwareInterface"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GHandHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("GHandHardwareInterface"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("GHandHardwareInterface"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type GHandHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO: Read actual hardware state via EtherCAT/RS485/CANFD
  // For now, mock: position follows command; diagnostics and tactile are zero-filled.
  for (size_t i = 0; i < hw_joint_states_.size(); ++i)
  {
    if (!std::isnan(hw_joint_commands_[i].position))
    {
      hw_joint_states_[i].position = hw_joint_commands_[i].position;
    }
    if (!std::isnan(hw_joint_commands_[i].velocity))
    {
      hw_joint_states_[i].velocity = hw_joint_commands_[i].velocity;
    }
    if (!std::isnan(hw_joint_commands_[i].effort))
    {
      hw_joint_states_[i].effort = hw_joint_commands_[i].effort;
    }
    hw_joint_states_[i].status = 0.0;
    hw_joint_states_[i].error = 0.0;
  }

  for (auto & finger : hw_finger_tactile_data_)
  {
    finger.resultant_force.x = 0.0;
    finger.resultant_force.y = 0.0;
    finger.resultant_force.z = 0.0;
    for (auto & force : finger.distributed_forces)
    {
      force.x = 0.0;
      force.y = 0.0;
      force.z = 0.0;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GHandHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Fill default values for missing command interfaces according to control mode.
  // Controllers only write the command interfaces they claim; unclaimed ones stay
  // NaN, which lets us detect the active mode and apply the required defaults.
  for (size_t i = 0; i < hw_joint_commands_.size(); ++i)
  {
    double & position = hw_joint_commands_[i].position;
    double & velocity = hw_joint_commands_[i].velocity;
    double & effort = hw_joint_commands_[i].effort;

    const bool has_position = !std::isnan(position);
    const bool has_velocity = !std::isnan(velocity);
    const bool has_effort = !std::isnan(effort);

    if (!has_position && !has_velocity && !has_effort)
    {
      // No active command yet: keep the hand idle.
      position = 0.0;
      velocity = 0.0;
      effort = 0.0;
    }
    else if (has_position && has_velocity && !has_effort)
    {
      // Position + velocity mode: effort defaults to 100%.
      effort = 100.0;
    }
    else if (has_position && has_effort && !has_velocity)
    {
      // Position + effort mode: velocity follows effort.
      velocity = effort;
    }
    else if (!has_position && has_velocity && !has_effort)
    {
      // Velocity mode: effort defaults to 100%.
      effort = 100.0;
    }
    // Effort-only mode: pass effort through, leave the rest at safe defaults below.

    if (std::isnan(position)) { position = 0.0; }
    if (std::isnan(velocity)) { velocity = 0.0; }
    if (std::isnan(effort)) { effort = 0.0; }
  }

  // TODO: Write hw_joint_commands_ to hardware via EtherCAT/RS485/CANFD
  return hardware_interface::return_type::OK;
}

}  // namespace GHand_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(GHand_hardware::GHandHardwareInterface, hardware_interface::SystemInterface)
