#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
using std::placeholders::_1;

/*
  Forwarding simulation clock bridged at
  /world/empty/clock to /clock
*/

class SimTimeForward : public rclcpp::Node
{
public:
  SimTimeForward()
  : Node("sim_time_forward")
  {
    publisher =
      this->create_publisher
        <rosgraph_msgs::msg::Clock>("/clock", 10);

    subscription =
      this->create_subscription
        <rosgraph_msgs::msg::Clock>
        ("/world/empty/clock", 10, 
          std::bind(&SimTimeForward::sub_callback, this, _1)
        );

  }

private:
  void sub_callback(const rosgraph_msgs::msg::Clock & msg) const
  { publisher->publish(msg); }
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