/*
  DEPRECATED, since odometry data is so far only used
  in autonomous feature, this functionality is merged into
  the motion_navigator node. 

  Calculating odometry based on 3 rotary encoders readings. 
*/

// C++ std libs
#include <memory>
#include <cmath>

// RCLCPP utilities
#include "rclcpp/rclcpp.hpp"

// Messages
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"

// Function to check for suffix in a string
bool hasSuffix (std::string const &fullString, std::string const &suffix) {
  if (fullString.length() >= suffix.length()) {
    return (0 == fullString.compare (fullString.length() - suffix.length(), suffix.length(), suffix));
  } else { return false; }
}

class EncoderToOdometry : public rclcpp::Node
{
  public:
    EncoderToOdometry() : Node("encoder_to_odometry")
    {
      RCLCPP_INFO(this->get_logger(), "\033[32mStarting Encoder to Odometry node\033[0m");
      
      // Odometry publisher
      odom_publisher_ = this->create_publisher
        <nav_msgs::msg::Odometry>("/odometry", 10);

      /*
        Joint states callback, this is the callback function that calculates Odometry
        based on 3 rotary encoder readings.
        Check README for the algorithm.
      */
      auto jointStatesCallback =
        [this](sensor_msgs::msg::JointState::UniquePtr states) -> void {
          for (long unsigned int i = 0; i < states->name.size(); i++) {

            const double enc_to_m {states->position[i] * 0.1}; // 0.1 is rotation angle to distance
            if (hasSuffix(states->name[i], "omni_joint")) {

              // Convert rotation angle to distance traveled
              switch (states->name[i][4]) {

                case 'f': // x_left
                  x_left = -enc_to_m; break;
                case 'g': // x_right
                  x_right = enc_to_m; break;
                case 'o': // y_front
                  y_front = -enc_to_m; break;

              }
            }
          }

          // Save encoder readings for next loop
          old_x_left = x_left;
          old_x_right = x_right;
          old_y_front = y_front;

          // ODOMETRY
          XY_ZRotate local_coords_update;

          // Lowering detail to compensate for noise
          float delta_x1 = x_left - old_x_left;
          float delta_x2 = x_right - old_x_right;
          float delta_y = y_front - old_y_front;

          // Calculating local coordinates update
          local_coords_update.x = (delta_x1 + delta_x2) / 2;
          // reversed z update to fix reversed orientation issu
          local_coords_update.z = (delta_x2 - delta_x1) / (enc_x_to_origin * 2) * -1;
          local_coords_update.y = delta_y - enc_y_to_origin * local_coords_update.z;

          // Applying local coordinates update to field coordinates
          field_coords.x += local_coords_update.x * cos(field_coords.z) - local_coords_update.y * sin(field_coords.z);
          field_coords.y += local_coords_update.x * sin(field_coords.z) + local_coords_update.y * cos(field_coords.z);
          field_coords.z += local_coords_update.z;

          // Odometry message building
          nav_msgs::msg::Odometry odometry;
          odometry.header.stamp = states->header.stamp;
          odometry.header.frame_id = "odom";
          odometry.child_frame_id = "base_link";

          odometry.pose.pose.position.x = field_coords.x;
          odometry.pose.pose.position.y = field_coords.y;
          odometry.pose.pose.position.z = 0.15;

          tf2::Quaternion quater;
          quater.setRPY(0, 0, field_coords.z);
          odometry.pose.pose.orientation.z = quater.z();
          odometry.pose.pose.orientation.y = quater.y();
          odometry.pose.pose.orientation.x = quater.x();
          odometry.pose.pose.orientation.w = quater.w();

          odometry.twist.twist.linear.x
          = odometry.twist.twist.linear.y
          = odometry.twist.twist.linear.z
          = odometry.twist.twist.angular.x 
          = odometry.twist.twist.angular.y
          = odometry.twist.twist.angular.z 
          = 0;

          odom_publisher_->publish(odometry);;
        };

      encoder_states_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, jointStatesCallback);
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr encoder_states_subscriber_; // Joint states subscriber object
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_; // Odometry publisher object

    struct XY_ZRotate
    {
      double x;
      double y;
      double z;
    };
    
    double x_left {}, x_right {}, y_front {}; // Encoder readings
    double old_x_left {}, old_x_right {}, old_y_front {}; // Encoder readings from previous loop
    const float enc_x_to_origin {0.4}; // Distance between encoder on x axis to robot local origin
    const float enc_y_to_origin {0.4}; // Distance between encoder on y axis to robot local origin
    XY_ZRotate field_coords = {0, 0, 0}; // Initialize coordinates relative to the field (global coords)
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EncoderToOdometry>());
  rclcpp::shutdown();
  return 0;
}