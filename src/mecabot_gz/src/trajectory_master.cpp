/*

  This node moves robot on a certain trajectory
  specified in the command topic. Currently supports:
  
  - Odometry-based point to point motion:
    Moves robot to goal coordinate by tracking the
    robot's motion with rotary encoders feedback and 
    maintain its trajectory using proportional control.

*/

#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "control_toolbox/pid.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/time.h"

class TrajectoryMaster : public rclcpp::Node 
{
public:
  TrajectoryMaster() : rclcpp::Node("trajectory_master") {
    RCLCPP_INFO(this->get_logger(), "\033[32mStarting Trajectory Master\033[0m");

    this->declare_parameter<float>("pid_kp", 0);
    this->declare_parameter<float>("pid_ki", 0);
    this->declare_parameter<float>("pid_kd", 0);
    this->declare_parameter<float>("pid_i_max", 0);
    this->declare_parameter<float>("pid_i_min", 0);

    this->declare_parameter<std::string>("msg_on_idle", IDLE);
    this->declare_parameter<std::string>("msg_on_run", RUN);
    this->declare_parameter<std::string>("msg_on_done", DONE);

    command_listener_ = this->create_subscription<std_msgs::msg::String>("/trajectory_command", 1, 
    std::bind(&TrajectoryMaster::commandParser_, this, std::placeholders::_1));

    command_status_ = this->create_publisher<std_msgs::msg::String>("/motion_status", 10);
    kinematics_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

    traj_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
    std::bind(&TrajectoryMaster::trajectoryExecutor_, this));

    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    pid_.initPid(0, 0, 0, 0, 0, false);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr kinematics_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_listener_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_status_;
  rclcpp::TimerBase::SharedPtr traj_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  control_toolbox::Pid pid_;

  // struct, so this can easily be replaced with custom messages
  struct CommandArgs {
    std::string command;
    std::vector<std::string> str_args;
    std::vector<float> num_args;
  };

  CommandArgs cmd_args_;
  float old_tf_time_{};
  float point_goal_x_{}, point_goal_y_{}, point_goal_theta_{};
  const std::string IDLE{"idle"}, RUN{"in_motion"}, DONE{"arrived"};

  void trajectoryExecutor_();
  void commandParser_(std_msgs::msg::String::UniquePtr);
  void cmdStatPub_(std::string);
};

void TrajectoryMaster::commandParser_(std_msgs::msg::String::UniquePtr command)
{
  CommandArgs empty = {"", {}, {}};  
  cmd_args_ = empty;

  try {
    std::string cmd_string = command->data;
    size_t first_slash = cmd_string.find('/');
    size_t last_slash = cmd_string.rfind('/');

    cmd_args_.command = cmd_string.substr(0, first_slash);
    std::stringstream str_args(cmd_string.substr(first_slash + 1, last_slash - first_slash - 1));
    std::stringstream num_args(cmd_string.substr(last_slash + 1));
    std::string token;

    while(std::getline(str_args, token, ',')) {cmd_args_.str_args.push_back(token);}
    while(std::getline(num_args, token, ',')) {cmd_args_.num_args.push_back(std::stof(token));}
  } catch (const std::exception &e) { RCLCPP_ERROR(this->get_logger(), "Invalid command message"); return;}

  if (cmd_args_.command == "point") {

    if (cmd_args_.str_args.empty()) {
      cmd_args_.command = "idle";
      RCLCPP_ERROR(
        this->get_logger(), 
        "%s command cannot have empty string argument, returning to idle",
        cmd_args_.command.c_str());
      return;
    }

    if (cmd_args_.num_args.empty()) {
      cmd_args_.command = "idle";
      RCLCPP_ERROR(
        this->get_logger(), 
        "%s command cannot have empty numeric argument, returning to idle",
        cmd_args_.command.c_str());
      return;
    }

    geometry_msgs::msg::TransformStamped absolute_coords;

    try {
      absolute_coords = tf2_buffer_->lookupTransform(
      "world", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(), "Could not find transforms from %s to %s",
        "base_link", "world");
      return;
    }

    // converts quaternion rotation to euler to get the yaw
    auto rotation = absolute_coords.transform.rotation;
    float absolute_yaw = 
      std::atan2(2.0 * (rotation.w * rotation.z + rotation.x * rotation.y), 
      1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z));

    std::string reference_mode = cmd_args_.str_args.at(0);
    auto absolute_translation = absolute_coords.transform.translation;

    if (reference_mode == "abs") {
      point_goal_x_ = cmd_args_.num_args.at(0);
      point_goal_y_ = cmd_args_.num_args.at(1);
      point_goal_theta_ = cmd_args_.num_args.at(2);

    } else if (reference_mode == "rel") {
      // rotated clockwise to follow robot's relative coordinate reference
      point_goal_x_ = cmd_args_.num_args.at(0) 
      + absolute_translation.x * cos(absolute_yaw) + absolute_translation.y * -sin(absolute_yaw);

      point_goal_y_ = cmd_args_.num_args.at(1)
      + absolute_translation.x * sin(absolute_yaw) + absolute_translation.y * cos(absolute_yaw);

      point_goal_theta_ = cmd_args_.num_args.at(2) + absolute_yaw;

    } else {
      cmd_args_.command = "idle";
      RCLCPP_WARN(
        this->get_logger(), 
        "%s command have invalid reference mode set, returning to idle",
        cmd_args_.command.c_str());
      return;
    }

    float p{}, i{}, d{}, i_max{}, i_min{};
    this->get_parameter("pid_kp", p);
    this->get_parameter("pid_ki", i);
    this->get_parameter("pid_kd", d);
    this->get_parameter("pid_i_max", i_max);
    this->get_parameter("pid_i_min", i_min); 
    pid_.setGains(p, i, d, i_max, i_min);

    RCLCPP_INFO(this->get_logger(), 
      "\033[43m\033[37m   In Motion   \033[0m\n[Point to Point] ref: %s x: %f y: %f yaw: %f", 
      reference_mode.c_str(), 
      point_goal_x_,
      point_goal_y_, 
      point_goal_theta_
    );

    cmdStatPub_(RUN);

  } else if (cmd_args_.command == "idle") {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.angular.z = 0;

    kinematics_pub_->publish(msg);
    
    RCLCPP_INFO(this->get_logger(), "\033[43m\033[37m   Idling...   \033[0m");
    cmdStatPub_(IDLE);
  } else {
    RCLCPP_WARN(
      this->get_logger(), 
      "Command %s is unknown",
      cmd_args_.command.c_str());
  }
}

void TrajectoryMaster::trajectoryExecutor_()
{
  if (cmd_args_.command == "point") {

    geometry_msgs::msg::TransformStamped absolute_coords;
    
    try {
      absolute_coords = tf2_buffer_->lookupTransform(
      "world", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(), "Could not find transforms from %s to %s",
        "base_link", "world");
      return;
    }

    // converts quaternion rotation to euler to get the yaw
    auto rotation = absolute_coords.transform.rotation;
    float absolute_yaw = 
      std::atan2(2.0 * (rotation.w * rotation.z + rotation.x * rotation.y), 
      1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z));

    const double EPSILON = 1e-4;
    double delta_x = point_goal_x_ - absolute_coords.transform.translation.x;
    double delta_y = point_goal_y_ - absolute_coords.transform.translation.y;
    double delta_theta = point_goal_theta_ - absolute_yaw;
    uint64_t delta_time = absolute_coords.header.stamp.nanosec - old_tf_time_;
  
    geometry_msgs::msg::Twist msg;
    if (abs(delta_x) > EPSILON || abs(delta_y) > EPSILON || abs(delta_theta) > EPSILON)
    {
      if (old_tf_time_ > 0) {
        double velX = pid_.computeCommand(delta_x, delta_time);
        double velY = pid_.computeCommand(delta_y, delta_time);

        // rotated ccw to maintain absolute reference motion at any robot yaw angle
        msg.linear.x = velX * cos(absolute_yaw) + velY * sin(absolute_yaw);
        msg.linear.y = -velX * sin(absolute_yaw) + velY * cos(absolute_yaw);
        msg.angular.z = pid_.computeCommand(delta_theta, delta_time);

        RCLCPP_INFO(this->get_logger(), "[PID] delta_x: %f delta_y: %f delta_theta: %f",
          delta_x, delta_y, delta_theta);
        
      } old_tf_time_ = absolute_coords.header.stamp.nanosec;

    } else {
      msg.linear.x = 0;
      msg.linear.y = 0;
      msg.angular.z = 0;
      cmd_args_.command = "idle";
      old_tf_time_ = 0;

      RCLCPP_INFO(this->get_logger(), "\033[42m\033[37m   Arrived on Target   \033[0m");
      cmdStatPub_(DONE);
    }

    kinematics_pub_->publish(msg);
  
  }
}

void TrajectoryMaster::cmdStatPub_(std::string status)
{
  std_msgs::msg::String stat;
  stat.data = status;
  command_status_->publish(stat);
}

int main(int argc, char** argv) 
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryMaster>());
  rclcpp::shutdown();
  return 0;
}