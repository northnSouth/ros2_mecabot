#include <chrono>
#include <memory>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
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

    // Publisher object
    publish_move_ =
      this->create_publisher
        <std_msgs::msg::Float64MultiArray>("/velo_c/commands", 10);

    // TF2 listener
    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    // Kinematics callback
    auto kinematics_callback =
      [this](geometry_msgs::msg::Twist::UniquePtr twist) -> void {
        
        // Get parameters
        bool global_ref;
        float speed_multiplier;
        this->get_parameter("global_ref", global_ref);
        this->get_parameter("speed_multiplier", speed_multiplier);

        // Rotated coords used if global reference mode is true
        double rotated_coords[2] = {twist->linear.x, twist->linear.y};

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
            angle = 2 * atan2(tf2.transform.rotation.z, tf2.transform.rotation.w);
          } catch (const tf2::TransformException & ex) {
            RCLCPP_ERROR(this->get_logger(),
              "Could not get base_link to world transforms %s", ex.what());
          }

          // Rotate input coords
          if (tf2_available) {
            rotated_coords[0] =
              twist->linear.x * cos(angle) + twist->linear.y * sin(angle);
            rotated_coords[1] =
              -twist->linear.x * sin(angle) + twist->linear.y * cos(angle);
          }
        }

        // Calculate polar coordinate from the cartesian input
        float vptheta = atan2(rotated_coords[1], rotated_coords[0]) + M_PI/4;
        float vpmagni = hypot(rotated_coords[0], rotated_coords[1]);

        // Caculate two mecanum wheel pair vectors
        float vector0 = vpmagni * cos(vptheta);
        float vector1 = vpmagni * sin(vptheta);

        // Setting motor speeds
        float m1 = vector0 + twist->angular.z;
        float m2 = vector1 - twist->angular.z;
        float m3 = vector1 + twist->angular.z;
        float m4 = vector0 - twist->angular.z;

        // If the sum vector magnitude goes above one, turn it down
        float summagni = vpmagni + abs(twist->angular.z);
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

    // CONTROLLER WAKE-UP SEQUENCE
    msg_.data = {0.0, 0.0, 0.0, 0.0};
    publish_move_->publish(msg_);
    RCLCPP_INFO(this->get_logger(), "\033[33mWaking up controller\033[0m");
    rclcpp::sleep_for(std::chrono::seconds(3));
    RCLCPP_INFO(this->get_logger(), "\033[92mController ready!\033[0m");

    // Subscriber object
    subscribe_move_ = 
      this->create_subscription
        <geometry_msgs::msg::Twist>("/cmd_vel", 10, kinematics_callback);
  }
private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publish_move_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscribe_move_;
  std_msgs::msg::Float64MultiArray msg_;
  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main (int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecabotMoveNode>());
  rclcpp::shutdown();
  return 0;
}