/*

  This node is used to create a directed map as static transforms
  of frames which are landmarks on the map. The map itself is generated
  based of a YAML file in the package's config directory. The map is
  a series of parent/child frames.

  TODO: Create a safer YAML parsing implementation

*/

// C++ std libs
#include <memory>
#include <string>

// RCLCPP utilities
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

// Messages
#include "geometry_msgs/msg/transform_stamped.hpp"

// TF2 utilities
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

// YAML processor library
#include "yaml-cpp/yaml.h"

class DirectedMapBroadcaster : public rclcpp::Node
{
public:
  explicit DirectedMapBroadcaster()
  : Node("directed_map_broadcaster")
  {
    RCLCPP_INFO(this->get_logger(), "\033[32mBroadcasting Directed Map\033[0m");

    // Static transforms broadcaster
    static_tf_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // YAML map config file parser
    for (const auto& point : map_config_["relations"]) {
      std::string pointName = point.as<std::string>();
      std::string parentFrame = pointName.substr(0, pointName.find('/'));
      std::string childFrame = pointName.substr(pointName.find('/') + 1, pointName.length());

      // Map broadcast
      this->broadcast_map(
        parentFrame, // Parent frame
        childFrame, // Child frame
        map_config_[childFrame][0].as<float>() - map_config_[parentFrame][0].as<float>(), // Child x coordinate
        map_config_[childFrame][1].as<float>() - map_config_[parentFrame][1].as<float>() // Child y coordinate
      );
    }
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_; // Static transforms broadcaster object
  YAML::Node map_config_ = YAML::LoadFile( // YAML file loader
    ament_index_cpp::get_package_share_directory("mecabot_gz")
    + "/config/directed_map.yaml"
  );

  // Map broadcaster function, basically builds a typical static transforms message
  void broadcast_map(std::string parent, std::string child, float x, float y) {
    geometry_msgs::msg::TransformStamped t;
    
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = parent;
    t.child_frame_id = child;

    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    t.transform.rotation.x = q.x(); 
    t.transform.rotation.y = q.y(); 
    t.transform.rotation.z = q.z(); 
    t.transform.rotation.w = q.w();

    static_tf_->sendTransform(t);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectedMapBroadcaster>());
  rclcpp::shutdown();
  return 0;
}