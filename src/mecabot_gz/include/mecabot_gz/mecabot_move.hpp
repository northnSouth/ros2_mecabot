#ifndef MECABOT_MOVE_HPP
#define MECABOT_MOVE_HPP

#include <controller_interface/controller_interface.hpp>
#include <controller_interface/controller_interface_base.hpp>
#include <rclcpp/duration.hpp>

/*
  Following ROS2 Control Jazzy documentation of
  step-by-step guide on controller creation

  Not doing visibility.h
  I don't use windows :)
*/

using namespace controller_interface;

namespace mecabot_gz {
  class MecabotMove : public ControllerInterface
  {
    public:
      MecabotMove();
      InterfaceConfiguration command_interface_configuration() const override;
      InterfaceConfiguration state_interface_configuration() const override;
      return_type update(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;
      CallbackReturn on_init() override;
      CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;
      CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;
      CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;
      CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;
      CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State & previous_state) override;
      CallbackReturn on_error(
        const rclcpp_lifecycle::State & previous_state) override;
  };
}

#endif