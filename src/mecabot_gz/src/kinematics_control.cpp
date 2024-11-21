/*

  This node is used to execute kinematics and move the robot
  (Gazebo 4-Wheel-Drive Mecanum robot) with Twist messages.
  See README for explanation of kinematics algorithm used.

*/

// C++ std libs
#include <chrono>
#include <memory>
#include <cmath>

// RCLCPP utilities
#include "rclcpp/rclcpp.hpp"

// Messages
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class MecabotMoveNode : public rclcpp::Node
{
public:
  MecabotMoveNode()
  : Node("kinematics_control")
  {
    RCLCPP_INFO(this->get_logger(), "\033[32mStarting Kinematics Control\033[0m");
    
    // Declare parameter
    this->declare_parameter<float>("speed_multiplier", 1);

    // Commands publisher
    publish_move_ =
      this->create_publisher
        <std_msgs::msg::Float64MultiArray>("/velo_c/commands", 10);
    
    // Twist message subscriber
    subscribe_move_ = 
      this->create_subscription
        <geometry_msgs::msg::Twist>("/cmd_vel", 10, 
        [this](geometry_msgs::msg::Twist::UniquePtr twist) -> void {
          twist_msg_ = *twist;
      });
      
    // Kinematics callback
    auto kinematics_callback =
      [this]() -> void {
        
        // Get parameter
        float speed_multiplier;
        this->get_parameter("speed_multiplier", speed_multiplier);

        // Calculate polar coordinate from the cartesian input
        float vptheta = atan2(this->twist_msg_.linear.y, this->twist_msg_.linear.x) + M_PI/4;
        float vpmagni = hypot(this->twist_msg_.linear.x, this->twist_msg_.linear.y);

        // Caculate two mecanum wheel pair vectors
        float vector0 = vpmagni * cos(vptheta);
        float vector1 = vpmagni * sin(vptheta);

        // Calculating motor speeds
        float m1 = vector0 - this->twist_msg_.angular.z;
        float m2 = vector1 + this->twist_msg_.angular.z;
        float m3 = vector1 - this->twist_msg_.angular.z;
        float m4 = vector0 + this->twist_msg_.angular.z;

        // If the sum vector magnitude goes above one, turn it down
        float summagni = vpmagni + abs(this->twist_msg_.angular.z);
        if (summagni > 1) {
          m1 /= summagni;
          m2 /= summagni;
          m3 /= summagni;
          m4 /= summagni;
        }

        // Builds message
        veloc_msg_.data = {
          // Left Front
          m1 * speed_multiplier,
          // Right Front
          m2 * speed_multiplier,
          // Left Rear
          m3 * speed_multiplier,
          // Right Rear
          m4 * speed_multiplier
        };

        // Publish message
        publish_move_->publish(veloc_msg_);

      };

    kinematics_worker_ = this->create_wall_timer(1ms, kinematics_callback);

    /*
      This part publishes stationary motion message to the controller
      in order to get the controller up and running. For some reason
      the controller doesn't respond to the first message, and ony responds
      to the second message after a few seconds (3) since the first one.
    */
    veloc_msg_.data = {0.0, 0.0, 0.0, 0.0};
    publish_move_->publish(veloc_msg_);
    RCLCPP_INFO(this->get_logger(), "\033[33mWaking up controller\033[0m");
    rclcpp::sleep_for(std::chrono::seconds(3));
    RCLCPP_INFO(this->get_logger(), "\033[92mController ready!\033[0m");
  }
  
private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publish_move_; // Controller publisher
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscribe_move_; // Twist listener
  rclcpp::TimerBase::SharedPtr kinematics_worker_; // High frequency kinematics calculator
  std_msgs::msg::Float64MultiArray veloc_msg_; // Controller message interface
  geometry_msgs::msg::Twist twist_msg_; // Twist message interface
  
};

int main (int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecabotMoveNode>());
  rclcpp::shutdown();
  return 0;
}