/*

  This node creates odometry data from sensors
  then broadcasts the data to /tf as world to
  base_link transform.

  Currently calculates odometry from 3 rotary
  encoders, algorithm description in README

*/

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class OdometryWorker : public rclcpp::Node
{
public:
  OdometryWorker() : Node("odometry_worker") {
    RCLCPP_INFO(this->get_logger(), "\033[32mStarting Odometry Worker\033[0m");
  
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 1, 
      std::bind(&OdometryWorker::jointStatesCallback_, this, std::placeholders::_1));

    tf2_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;

  struct XYTheta {
    float x;
    float y;
    float theta;
  };

  float old_x_left_{}, old_x_right_{},old_y_front_{};
  const float enc_x_to_center_{0.4};
  const float enc_y_to_center_{0.4};
  XYTheta global_coords_{0, 0, 0};

  void jointStatesCallback_(sensor_msgs::msg::JointState::UniquePtr);
};

bool hasSuffix (std::string const &fullString, std::string const &suffix) 
{
  if (fullString.length() >= suffix.length()) {
    return (0 == fullString.compare (fullString.length() - suffix.length(), suffix.length(), suffix));
  } else { return false; }
}

void OdometryWorker::jointStatesCallback_(sensor_msgs::msg::JointState::UniquePtr states)
{
  float x_left {}, x_right {}, y_front {};
  for (long unsigned int i = 0; i < states->name.size(); i++) {
    const double enc_to_m {states->position[i] * 0.1}; // did the math, 0.1 is the right value
    
    if (hasSuffix(states->name[i], "omni_joint")) {
      switch (states->name[i][4]) {
        // x_left and y_front negated so the coordinate reference is the same
        case 'f': 
          x_left = -enc_to_m; break;
        case 'g': 
          x_right = enc_to_m; break;
        case 'o': 
          y_front = -enc_to_m; break;
      }
    }
  }

  XYTheta local_coords_update;

  local_coords_update.x = (
    (x_left - old_x_left_) + // delta_X1 
    (x_right - old_x_right_) // delta_X2
  ) / 2;

  local_coords_update.theta = (
    (x_right - old_x_right_) - // delta_X2 
    (x_left - old_x_left_) // delta_X1
  ) / (enc_x_to_center_ * 2) * -1; // negate to fix reversed orientation issue

  local_coords_update.y =
    y_front - old_y_front_ - // delta_Y 
    enc_y_to_center_ * local_coords_update.theta;

  old_x_left_ = x_left;
  old_x_right_ = x_right;
  old_y_front_ = y_front;

  global_coords_.theta += local_coords_update.theta;
  global_coords_.x += local_coords_update.x 
    * cos(global_coords_.theta) - local_coords_update.y * sin(global_coords_.theta);
  global_coords_.y += local_coords_update.x
    * sin(global_coords_.theta) + local_coords_update.y * cos(global_coords_.theta);

  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = states->header.stamp;
  t.header.frame_id = "world";
  t.child_frame_id = "base_link";

  t.transform.translation.x = global_coords_.x;
  t.transform.translation.y = global_coords_.y;
  t.transform.translation.z = 0.15;

  tf2::Quaternion q;
  q.setRPY(0, 0, global_coords_.theta);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  tf2_broadcaster_->sendTransform(t);

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryWorker>());
  rclcpp::shutdown();
  return 0;
}