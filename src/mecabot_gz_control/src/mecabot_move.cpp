#include "mecabot_gz_control/mecabot_move.hpp"
#include <controller_interface/controller_interface.hpp>
#include <controller_interface/controller_interface_base.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <pluginlib/class_list_macros.hpp>

// Using namespaces for readability
using namespace mecabot_move;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;
using controller_interface::CallbackReturn;
using controller_interface::return_type;

// This fixes failed loading test
MecabotMove::MecabotMove()
  : controller_interface::ControllerInterface() {}

InterfaceConfiguration 
  MecabotMove::command_interface_configuration() const
{
  /*
    Here configuring command interfaces used
  */
  return {interface_configuration_type::NONE};
}

InterfaceConfiguration 
  MecabotMove::state_interface_configuration() const
{
  /*
    Here configuring state interfaces used
  */
  return {interface_configuration_type::NONE};
}

CallbackReturn MecabotMove::on_init()
{
  /* 
    Step 0, code here run in uninitialized state
  */
  return CallbackReturn::SUCCESS;
}

CallbackReturn MecabotMove::on_configure(
  const rclcpp_lifecycle::State &)
{
  /*
    Step 1, code here run when the controller is
    transitioning from uninitialized to inactive
  */
  return CallbackReturn::SUCCESS;
}

CallbackReturn MecabotMove::on_activate(
  const rclcpp_lifecycle::State &)
{
  /*
    Step 2, code here run when the controller is
    transitioning from inactive to active
  */
  return CallbackReturn::SUCCESS;
}

return_type MecabotMove::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  /*
    Step 3, code here run when the controller is
    running normally
  */
  return return_type::OK;
}

CallbackReturn MecabotMove::on_error(const rclcpp_lifecycle::State &)
{
  /*
    Error, code here run when the controller is
    having an error
  */
  return CallbackReturn::SUCCESS;
}

CallbackReturn MecabotMove::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  /*
    Step 4, code here run when the controller is
    transitioning from active to inactive
  */ 
  return CallbackReturn::SUCCESS;
}

CallbackReturn MecabotMove::on_cleanup(const rclcpp_lifecycle::State &)
{
  /*
    Step 5, ambiguous, might run when controller goes
    from inactive to 'finalized'
  */
  return CallbackReturn::SUCCESS;
}

CallbackReturn MecabotMove::on_shutdown(const rclcpp_lifecycle::State &)
{
  /*
    Step 6, ambiguous, might run when controller goes
    from any state to 'finalized'
  */
  return CallbackReturn::SUCCESS;
}

PLUGINLIB_EXPORT_CLASS(
  mecabot_move::MecabotMove, controller_interface::ControllerInterface
)