/*

  This node is used to execute kinematics and move the robot
  (Gazebo 4-Wheel-Drive Mecanum robot) with Twist messages.
  See README for explanation of kinematics algorithm used.

*/

#include <chrono>
#include <functional>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

class KinematicsControl : public rclcpp::Node
{
public:
  KinematicsControl() : Node("kinematics_control") {
    RCLCPP_INFO(this->get_logger(), "\033[32mStarting Kinematics Control\033[0m");
    
    this->declare_parameter<float>("speed_multiplier", 1);

    motion_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/velo_c/commands", 10);
    command_listener_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, 
      [this](geometry_msgs::msg::Twist::UniquePtr twist) { twist_msg_ = *twist; });

    kinematics_worker_ = this->create_wall_timer(std::chrono::milliseconds(10), 
    std::bind(&KinematicsControl::kinematicsRoutine_, this));

    // controller only works from the second message published, idk why
    veloc_msg_.data = {0.0, 0.0, 0.0, 0.0};
    motion_publisher_->publish(veloc_msg_);
    RCLCPP_INFO(this->get_logger(), "\033[33mWaking up controller\033[0m");
    rclcpp::sleep_for(std::chrono::seconds(3));
    RCLCPP_INFO(this->get_logger(), "\033[92mController ready!\033[0m");
  }
  
private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motion_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_listener_; 
  rclcpp::TimerBase::SharedPtr kinematics_worker_; 
  std_msgs::msg::Float64MultiArray veloc_msg_; 
  geometry_msgs::msg::Twist twist_msg_;

  void kinematicsRoutine_();
};

void KinematicsControl::kinematicsRoutine_()
{
  float speed_multiplier;
  this->get_parameter("speed_multiplier", speed_multiplier);

  // calculate polar coordinates
  float vptheta = atan2(this->twist_msg_.linear.y, this->twist_msg_.linear.x) + M_PI/4;
  float vpmagni = hypot(this->twist_msg_.linear.x, this->twist_msg_.linear.y);

  float vector0 = vpmagni * cos(vptheta);
  float vector1 = vpmagni * sin(vptheta);

  float m1 = vector0 - this->twist_msg_.angular.z;
  float m2 = vector1 + this->twist_msg_.angular.z;
  float m3 = vector1 - this->twist_msg_.angular.z;
  float m4 = vector0 + this->twist_msg_.angular.z;

  // prevents output signal above 1
  float summagni = vpmagni + abs(this->twist_msg_.angular.z);
  if (summagni > 1) {
    m1 /= summagni;
    m2 /= summagni;
    m3 /= summagni;
    m4 /= summagni;
  }

  veloc_msg_.data = {
    m1 * speed_multiplier, // left front
    m2 * speed_multiplier, // right front
    m3 * speed_multiplier, // left rear
    m4 * speed_multiplier // right rear
  };
    
  motion_publisher_->publish(veloc_msg_);
}

int main (int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicsControl>());
  rclcpp::shutdown();
  return 0;
}