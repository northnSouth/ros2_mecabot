#include <memory>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

// Messages
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

/*
  Calculating odometry based on 3 
  passive rotary encoders readings 
*/

bool hasSuffix (std::string const&, std::string const&); // Check if a string has a certain suffix
bool hasPrefix (std::string const&, std::string const&); // Check if a string has a certain prefix


// Node class
class EncoderToOdometry : public rclcpp::Node
{
  public:
    EncoderToOdometry() : Node("encoder_to_odometry")
    {
      RCLCPP_INFO(this->get_logger(), "\033[32mStarting Encoder to Odometry node\033[0m");
      
      tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
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
                case 'a': // y_rear
                  y_rear = enc_to_m; break;

              }
            }
          }


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
          // Why x and y is decreased? It transformed backwards idk why. Too bad!
          field_coords.x += local_coords_update.x * cos(field_coords.z) - local_coords_update.y * sin(field_coords.z);
          field_coords.y += local_coords_update.x * sin(field_coords.z) + local_coords_update.y * cos(field_coords.z);
          field_coords.z += local_coords_update.z;

          // TRANSFORM MESSAGE BROADCASTER
          geometry_msgs::msg::TransformStamped tf_stamped;
          tf_stamped.header.stamp = states->header.stamp;
          tf_stamped.header.frame_id = "world";
          tf_stamped.child_frame_id = "base_link";

          tf_stamped.transform.translation.x = field_coords.x;
          tf_stamped.transform.translation.y = field_coords.y;
          tf_stamped.transform.translation.z = 0.15;

          tf2::Quaternion quater;
          quater.setRPY(0, 0, field_coords.z);
          tf_stamped.transform.rotation.z = quater.z();
          tf_stamped.transform.rotation.y = quater.y();
          tf_stamped.transform.rotation.x = quater.x();
          tf_stamped.transform.rotation.w = quater.w();

          tf_broadcaster->sendTransform(tf_stamped);

          // RCLCPP_INFO(this->get_logger(), "x: %lf", field_coords.x);
          // RCLCPP_INFO(this->get_logger(), "y: %lf", field_coords.y);
          // RCLCPP_INFO(this->get_logger(), "z: %lf", field_coords.z);

          // Save encoder readings for next loop
          old_x_left = x_left;
          old_x_right = x_right;
          old_y_front = y_front;
        };

      encoder_states_subscriber = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, jointStatesCallback);
    }

  private:
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr encoder_states_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transforms_publisher;

    struct XY_ZRotate // Structure for odometry calculation
    {
      double x;
      double y;
      double z;
    };
    
    double x_left {}, x_right {}, y_front {}, y_rear {}; // Encoder readings
    double old_x_left {}, old_x_right {}, old_y_front {}; // Encoder readings from previous loop
    const float enc_x_to_origin {0.15}; // Distance between encoder on x axis to robot local origin
    const float enc_y_to_origin {0.15}; // Distance between encoder on y axis to robot local origin
    XY_ZRotate field_coords = {0, 0, 0}; // Initialize coordinates relative to the field (global coords)
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EncoderToOdometry>());
  rclcpp::shutdown();
  return 0;
}


// FUNCTION BODIES
bool hasPrefix (std::string const &fullString, std::string const &prefix)
{
    if (fullString.length() >= prefix.length()) {
        return (0 == fullString.compare (0, prefix.length(), prefix));
    } else { return false; }
}

bool hasSuffix (std::string const &fullString, std::string const &suffix)
{
    if (fullString.length() >= suffix.length()) {
        return (0 == fullString.compare (fullString.length() - suffix.length(), suffix.length(), suffix));
    } else { return false; }
}