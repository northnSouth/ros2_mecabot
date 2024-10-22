#ifndef MECABOT_MOVE_HPP_
#define MECABOT_MOVE_HPP_

#include "controller_interface/controller_interface.hpp"
#include "controller_interface/controller_interface_base.hpp"
#include "visibility_control.h"

namespace mecabot_move {
  class MecabotMove
    : public controller_interface::ControllerInterface
  {
  public:
    MECABOT_MOVE_PUBLIC
    MecabotMove();

    MECABOT_MOVE_PUBLIC
    controller_interface::CallbackReturn on_init() override;

    MECABOT_MOVE_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    MECABOT_MOVE_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  
    MECABOT_MOVE_PUBLIC
    controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State & previous_state) override;

    MECABOT_MOVE_PUBLIC
    controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & previous_state) override;
  
    MECABOT_MOVE_PUBLIC
    controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & previous_state) override;
  
    MECABOT_MOVE_PUBLIC
    controller_interface::return_type update(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;
  };
}

#endif