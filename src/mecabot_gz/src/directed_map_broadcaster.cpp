/*

  This node is used to create a directed map as static transforms
  of frames which are landmarks on the map. The map itself is generated
  based of a YAML file in the package's config directory. The map is
  a series of parent/child frames.

*/

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "yaml-cpp/yaml.h"

class DirectedMapBroadcaster : public rclcpp::Node
{
public:
  DirectedMapBroadcaster() : Node("directed_map_broadcaster") {
    RCLCPP_INFO(this->get_logger(), "\033[32mBroadcasting Directed Map\033[0m");

    static_tf_broad_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    for (const auto& joint : map_config_["relations"]) {
      std::string jointPair = joint.as<std::string>();
      std::string parentFrame = jointPair.substr(0, jointPair.find('/'));
      std::string childFrame = jointPair.substr(jointPair.find('/') + 1, jointPair.length());

      this->broadcastMap_(
        parentFrame, 
        childFrame, 
        map_config_[childFrame][0].as<float>() - map_config_[parentFrame][0].as<float>(), 
        map_config_[childFrame][1].as<float>() - map_config_[parentFrame][1].as<float>() 
      );
    }
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broad_; 
  YAML::Node map_config_ = YAML::LoadFile( 
    ament_index_cpp::get_package_share_directory("mecabot_gz")
    + "/config/directed_map.yaml"
  );

  void broadcastMap_(std::string, std::string, float, float);
};

void DirectedMapBroadcaster::broadcastMap_(std::string parent, std::string child, float x, float y)
{
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

  static_tf_broad_->sendTransform(t);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectedMapBroadcaster>());
  rclcpp::shutdown();
  return 0;
}