#include "mecabot_gz_control/mecabot_move.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_interface/controller_interface_base.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "pluginlib/class_list_macros.hpp"

using namespace mecabot_move;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;
using controller_interface::CallbackReturn;
using controller_interface::return_type;

MecabotMove::MecabotMove()
  : controller_interface::ControllerInterface() {}

CallbackReturn MecabotMove::on_init()
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn MecabotMove::on_configure(
  const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration 
  MecabotMove::command_interface_configuration() const
{
  return {interface_configuration_type::NONE};
}

InterfaceConfiguration 
  MecabotMove::state_interface_configuration() const
{
  return {interface_configuration_type::NONE};
}

CallbackReturn MecabotMove::on_activate(
  const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn MecabotMove::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

return_type MecabotMove::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return return_type::OK;
}

PLUGINLIB_EXPORT_CLASS(
  mecabot_move::MecabotMove, controller_interface::ControllerInterface
)