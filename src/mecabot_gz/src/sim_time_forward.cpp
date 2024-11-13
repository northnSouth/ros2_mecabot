/*

  This node is used to forward simulation clock 
  bridged at /world/empty/clock to /clock

*/

// C++ std libs
#include "rosgraph_msgs/msg/clock.hpp"
#include <memory>

// RCLCPP utilities
#include <rclcpp/rclcpp.hpp>

// Message
#include <rosgraph_msgs/msg/clock.hpp>

class SimTimeForward : public rclcpp::Node
{
public:
  SimTimeForward()
  : Node("sim_time_forward")
  {
    RCLCPP_INFO(this->get_logger(), "\033[32mStarting Sim Time Forward node\033[0m");

    // Time publisher
    publisher =
      this->create_publisher
        <rosgraph_msgs::msg::Clock>("/clock", 10);

    // Time subscriber
    subscription =
      this->create_subscription
        <rosgraph_msgs::msg::Clock>
        ("/world/empty/clock", 10, 
        [this](const rosgraph_msgs::msg::Clock& msg) -> void 
          {publisher->publish(msg); }
        );
  }

private:
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr subscription; // Time subscriber object
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher; // Time publisher object
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimTimeForward>());
  rclcpp::shutdown();
  return 0;
}