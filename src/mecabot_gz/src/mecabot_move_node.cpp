#include <chrono>
#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <tf2/time.h>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"

using namespace std::chrono_literals;

/*
  Moves the robot with kinematics
  with respect to global or local
  coordinates
*/

class MecabotMoveNode : public rclcpp::Node
{
public:
  MecabotMoveNode()
  : Node("mecabot_move")
  {
    RCLCPP_INFO(this->get_logger(), "\033[32mStarting Mecabot Move node\033[0m");
    
    // Declare parameters
    this->declare_parameter<bool>("global_ref", false);
    this->declare_parameter<float>("speed_multiplier", 1);

    // Commands publisher
    publish_move_ =
      this->create_publisher
        <std_msgs::msg::Float64MultiArray>("/velo_c/commands", 10);
    
    // Twist subscriber
    subscribe_move_ = 
      this->create_subscription
        <geometry_msgs::msg::Twist>("/cmd_vel", 10, 
        [this](geometry_msgs::msg::Twist::UniquePtr twist) -> void {
          twist_msg_ = *twist;
      });

    // TF2 listener
    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
      
    // Kinematics callback
    auto kinematics_callback =
      [this]() -> void {
        
        // Get parameters
        bool global_ref;
        float speed_multiplier;
        this->get_parameter("global_ref", global_ref);
        this->get_parameter("speed_multiplier", speed_multiplier);

        // Motion direction in cartesian coordinates
        double cartesian_direction[2] = {this->twist_msg_.linear.x, this->twist_msg_.linear.y};

        // Global reference mode
        if (global_ref) {
          geometry_msgs::msg::TransformStamped tf2;
          bool tf2_available = false;
          float angle = 0;

          // Check if base_link transforms are available
          try {
            tf2 = tf2_buffer_->lookupTransform(
              "base_link", "world",
              tf2::TimePointZero);
            tf2_available = true;

            // Convert quaternion angle to euler
            angle = 2 * atan2(tf2.transform.rotation.z, tf2.transform.rotation.w) * -1;
          } catch (const tf2::TransformException & ex) {
            RCLCPP_ERROR(this->get_logger(),
              "Could not get base_link to world transforms %s", ex.what());
          }

          // Rotate input coords
          if (tf2_available) {
            cartesian_direction[0] =
              this->twist_msg_.linear.x * cos(angle) + this->twist_msg_.linear.y * sin(angle);
            cartesian_direction[1] =
              -this->twist_msg_.linear.x * sin(angle) + this->twist_msg_.linear.y * cos(angle);
          }
        }

        // Calculate polar coordinate from the cartesian input
        float vptheta = atan2(cartesian_direction[1], cartesian_direction[0]) + M_PI/4;
        float vpmagni = hypot(cartesian_direction[0], cartesian_direction[1]);

        // Caculate two mecanum wheel pair vectors
        float vector0 = vpmagni * cos(vptheta);
        float vector1 = vpmagni * sin(vptheta);

        // Setting motor speeds
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
        msg_.data = {
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
        publish_move_->publish(msg_);
      };

    kinematics_worker_ = this->create_wall_timer(1ms, kinematics_callback);

    // CONTROLLER WAKE-UP SEQUENCE
    msg_.data = {0.0, 0.0, 0.0, 0.0};
    publish_move_->publish(msg_);
    RCLCPP_INFO(this->get_logger(), "\033[33mWaking up controller\033[0m");
    rclcpp::sleep_for(std::chrono::seconds(3));
    RCLCPP_INFO(this->get_logger(), "\033[92mController ready!\033[0m");
  }
  
private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publish_move_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscribe_move_;
  rclcpp::TimerBase::SharedPtr kinematics_worker_;
  std_msgs::msg::Float64MultiArray msg_;
  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  geometry_msgs::msg::Twist twist_msg_;
  
};

int main (int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecabotMoveNode>());
  rclcpp::shutdown();
  return 0;
}