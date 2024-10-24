#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

/*
  Moves the robot with kinematics
  with respect of global or local
  coordinates
*/

class MecabotMoveNode : public rclcpp::Node
{
public:
  MecabotMoveNode()
  : Node("mecabot_move")
  {
    RCLCPP_INFO(this->get_logger(), "\033[32mStarting Mecabot Move node\033[0m");

    publish_move_ =
      this->create_publisher
        <std_msgs::msg::Float64MultiArray>("/velo_c/commands", 10);
  
    auto timer_callback =
      [this]() -> void {
        msg_.layout.data_offset = 0;
        msg_.layout.dim = {};
        msg_.data = {2.0, -2.0, 2.0, -2.0};

        publish_move_->publish(msg_);
      };

    timer_ = this->create_wall_timer(100ms, timer_callback);
  }
private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publish_move_;
  std_msgs::msg::Float64MultiArray msg_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main (int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecabotMoveNode>());
  rclcpp::shutdown();
  return 0;
}