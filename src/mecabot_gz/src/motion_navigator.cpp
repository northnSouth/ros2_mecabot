#include <algorithm>
#include <cmath>
#include <cstdint>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/timer.hpp>
#include <sstream>
#include <memory>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <exception>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "control_toolbox/pid.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class MotionNavigator : public rclcpp::Node
{
public:
  MotionNavigator() : Node("motion_navigator")
  {
    RCLCPP_INFO(this->get_logger(), "\033[32mStarting Motion Navigator\033[0m");
    RCLCPP_INFO(this->get_logger(), "\033[43m\033[37m   Idling...   \033[0m");

    // Node parameters declaration
    this->declare_parameter<float>("pid_kp", 15.0);
    this->declare_parameter<float>("pid_ki", 0);
    this->declare_parameter<float>("pid_kd", 0);
    this->declare_parameter<float>("pid_i_max", 0);
    this->declare_parameter<float>("pid_i_min", 0);

    // Pub/sub and broadcasters/listeners
    tf2_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    motion_publisher_ = this->create_publisher
      <geometry_msgs::msg::Twist>("/cmd_vel", 10);

    encoder_states_subscriber_ = this->create_subscription
      <sensor_msgs::msg::JointState>("/joint_states", 10, 
        std::bind(&MotionNavigator::encodersOdometry_, this, std::placeholders::_1)
      );

    /*
      Command listener/parser and motion initiation
      Example command:

      odometry/local/1,0,0
      laser/global/3.14

      -- First word is limiter mode
      -- The remaining words are limiter specific
      -- Case sensitive
    */

    command_listener_ = this->create_subscription
      <std_msgs::msg::String>("/motion_navigator", 10, 
      [this](std_msgs::msg::String msg) -> void {

        // Parsing command as string vector
        cmd_args_ = {};
        std::stringstream ss(msg.data);
        std::string token;
        while(std::getline(ss, token, '/')) {cmd_args_.push_back(token);}

        try {
          if (cmd_args_.at(0) == "odometry") { // Odometry limited state

            // PID controller gain setting
            float p{}, i{}, d{}, i_max{}, i_min{};
            this->get_parameter("pid_kp", p);
            this->get_parameter("pid_ki", i);
            this->get_parameter("pid_kd", d);
            this->get_parameter("pid_i_max", i_max);
            this->get_parameter("pid_i_min", i_min);

            pid_.setGains(p, i, d, i_max, i_min);

            // Parse goal coordinates
            std::vector<float>goal_coords_buffer = {};
            std::stringstream ss(cmd_args_.at(2).c_str());
            std::string token;
            while(std::getline(ss, token, ',')) {goal_coords_buffer.push_back(std::stof(token));}

            if (cmd_args_.at(1) == "local") {
              odometry_goal_.x = world_tfs_.transform.translation.x + goal_coords_buffer.at(0);
              odometry_goal_.y = world_tfs_.transform.translation.y + goal_coords_buffer.at(1);
              odometry_goal_.theta = world_yaw_ + goal_coords_buffer.at(2);
            } else if (cmd_args_.at(1) == "global") {
              odometry_goal_.x = goal_coords_buffer.at(0);
              odometry_goal_.y = goal_coords_buffer.at(1);
              odometry_goal_.theta = goal_coords_buffer.at(2);
            } else {
              RCLCPP_ERROR(this->get_logger(), "Second argument (coordinate reference) invalid, resetting to idle");
              cmd_args_ = {"idle"};
            }
          
            // Log odometry limited motion info
            RCLCPP_INFO(this->get_logger(), 
            "\033[42m\033[37m   In Motion   \033[0m\n[Odometry Limit] mode: %s x: %f y: %f yaw: %f", 
              cmd_args_.at(1).c_str(), 
              odometry_goal_.x, odometry_goal_.y, odometry_goal_.theta);
          
          } else if (cmd_args_.at(0) == "idle") { // Idle state
            RCLCPP_INFO(this->get_logger(), "\033[43m\033[37m   Idling...   \033[0m");
          } else if (cmd_args_.at(0) == "odom_reset") { // Reset odometry
            RCLCPP_INFO(this->get_logger(), "\033[43m\033[37m   Resetting Odometry...   \033[0m");
          } else { // Unknown command handler
            RCLCPP_ERROR(this->get_logger(), "Unknown command, resetting to idle");
            cmd_args_ = {"idle"};
          }
          
        } catch (const std::exception& e) { // Error handler
          RCLCPP_ERROR(this->get_logger(), "Snap! Command parser breaks: %s", e.what());}

      });

      // PID initialization
      pid_.initPid(0, 0, 0, 0, 0, false);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_listener_;
  std::vector<std::string> cmd_args_{"idle"}; // Parsed command arguments
  geometry_msgs::msg::Pose2D odometry_goal_; // Odometry goal on global reference
  geometry_msgs::msg::TransformStamped world_tfs_; // world to base_link transforms
  float world_yaw_; // Robot yaw/orientation on global reference
  
  control_toolbox::Pid pid_;
  uint64_t last_time_{}; // Time tracker for PID

  void encodersOdometry_(sensor_msgs::msg::JointState::UniquePtr states);
  double old_x_left_ {}, old_x_right_ {}, old_y_front_ {}; // Encoder readings from previous loop
  const float enc_x_to_origin_ {0.4}; // Distance between encoder on x axis to robot local origin
  const float enc_y_to_origin_ {0.4}; // Distance between encoder on y axis to robot local origin
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr encoder_states_subscriber_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motion_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;
};



/*
  /// Encoders to Odometry routine

  Algorithm definition is in README of the repository
  Reading 3 encoders one of which is perpendicular to
  the other two, to get a 2D pose of the robot (odometry)

  Here, odometry isn't used continuously, rather it is
  used only when a motion takes place. When in idle or
  beginning/end of a motion, odometry is reset
*/

// Function to check for suffix in a string
bool hasSuffix (std::string const &fullString, std::string const &suffix) {
  if (fullString.length() >= suffix.length()) {
    return (0 == fullString.compare (fullString.length() - suffix.length(), suffix.length(), suffix));
  } else { return false; }
}

// Function to force value to be zero if it is under a certain threshold
double dblForceZero (double dbl, double thresh) {
  return (std::fabs(dbl) < thresh) ? 0.0 : dbl;
}

void MotionNavigator::encodersOdometry_(sensor_msgs::msg::JointState::UniquePtr states)
{
  /*
    During laser limited motion(wall balance), world_tfs_ is overidden 
    by the balancing point coordinates, thus the encoders odometry based 
    world_tfs_ is suspended.
  */

  if (cmd_args_.at(0) != "laser") {

    /// ODOMETRY ACQUISITION BLOCK ///

    double x_left {}, x_right {}, y_front {}; // Real-time encoder readings
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

    /*
      old_* variables is updated during an odom_reset command since the 
      command is used to initialize odometry to zero and start reading 
      from the encoders' current state.
    */

    if (cmd_args_.at(0) == "odom_reset") {
      old_x_left_ = x_left;
      old_x_right_ = x_right;
      old_y_front_ = y_front;
    }

    // ODOMETRY
    geometry_msgs::msg::Pose2D local_coords_update;

    // Lowering detail to compensate for noise
    float delta_x1 = x_left - old_x_left_;
    float delta_x2 = x_right - old_x_right_;
    float delta_y = y_front - old_y_front_;

    // Calculating local coordinates update
    // Reversed theta update to fix reversed orientation issue
    local_coords_update.x = (delta_x1 + delta_x2) / 2;
    local_coords_update.theta = (delta_x2 - delta_x1) / (enc_x_to_origin_ * 2) * -1;
    local_coords_update.y = delta_y - enc_y_to_origin_ * local_coords_update.theta;

    // Save encoder readings for next iteration
    old_x_left_ = x_left;
    old_x_right_ = x_right;
    old_y_front_ = y_front;

    // Build transform messages
    world_tfs_.header.stamp = states->header.stamp;
    world_tfs_.header.frame_id = "world";
    world_tfs_.child_frame_id = "base_link";

    // Applying local coordinates update to world_tfs_ coordinates
    world_yaw_ += local_coords_update.theta;
    world_tfs_.transform.translation.x += 
      local_coords_update.x * cos(world_yaw_) - local_coords_update.y * sin(world_yaw_);
    world_tfs_.transform.translation.y += 
      local_coords_update.x * sin(world_yaw_) + local_coords_update.y * cos(world_yaw_);
    world_tfs_.transform.translation.z = 0.15;

    tf2::Quaternion q;
    q.setRPY(0, 0, world_yaw_);
    world_tfs_.transform.rotation.x = q.x();
    world_tfs_.transform.rotation.y = q.y();
    world_tfs_.transform.rotation.z = q.z();
    world_tfs_.transform.rotation.w = q.w();

    // Broadcast world to base_link transforms
    tf2_broadcaster_->sendTransform(world_tfs_);

    /// ODOMETRY MOTION BLOCK ///

    if (cmd_args_.at(0) == "odometry") {
      
      //These variables track changes in robot position relative to the goal coordinates
      const double DBL_CUTOFF = 1e-5;

      double deltaX = dblForceZero(odometry_goal_.x - world_tfs_.transform.translation.x, DBL_CUTOFF);
      double deltaY = dblForceZero(odometry_goal_.y - world_tfs_.transform.translation.y, DBL_CUTOFF);
      double deltaTheta = dblForceZero(odometry_goal_.theta - world_yaw_, DBL_CUTOFF);
      uint64_t deltaTime = states->header.stamp.nanosec - last_time_;

      RCLCPP_INFO(this->get_logger(), "%f %f %f", deltaX, deltaY, deltaTheta);

      /*
        This basically checks if the robot is close enough to the goal coords.
        because of simulator noise, i had to use these expressions to check if
        the value is larger than the DBL_CUTOFF
      */
      geometry_msgs::msg::Twist msg;
      if ( deltaX || deltaY || deltaTheta ) {
        // Robot in motion
        double velX = pid_.computeCommand(deltaX, deltaTime);
        double velY = pid_.computeCommand(deltaY, deltaTime);

        msg.linear.x = velX * cos(world_yaw_) + velY * sin(world_yaw_);
        msg.linear.y = -velX * sin(world_yaw_) + velY * cos(world_yaw_);
        msg.angular.z = pid_.computeCommand(deltaTheta, deltaTime);
      } else {
        // Robot stop
        msg.linear.x = 0;
        msg.linear.y = 0;
        msg.angular.z = 0;
        cmd_args_ = {"idle"};
        RCLCPP_INFO(this->get_logger(), 
          "\033[42m\033[37m   Arrived on Target   \033[0m\nx: %f y: %f yaw: %f",  
            odometry_goal_.x, odometry_goal_.y, odometry_goal_.theta);

      }

      motion_publisher_->publish(msg);
    }

    last_time_ = states->header.stamp.nanosec;
  }
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionNavigator>());
  rclcpp::shutdown();
  return 0;
}