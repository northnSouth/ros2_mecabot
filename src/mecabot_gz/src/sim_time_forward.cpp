/*

  This node is used to forward simulation clock 
  bridged at /world/empty/clock to /clock

*/

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "rosgraph_msgs/msg/clock.hpp"

class SimTimeForward : public rclcpp::Node
{
public:
  SimTimeForward() : Node("sim_time_forward") {
    RCLCPP_INFO(this->get_logger(), "\033[32mStarting Sim Time Forward node\033[0m");

    publisher = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    subscription = this->create_subscription<rosgraph_msgs::msg::Clock>("/world/empty/clock", 10, 
      [this](const rosgraph_msgs::msg::Clock& msg) { publisher->publish(msg); });
  }

private:
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr subscription;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimTimeForward>());
  rclcpp::shutdown();
  return 0;
}