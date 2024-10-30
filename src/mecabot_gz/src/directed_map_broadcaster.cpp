#include <memory>
#include <string>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "yaml-cpp/yaml.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

class DirectedMapBroadcaster : public rclcpp::Node
{
public:
  explicit DirectedMapBroadcaster()
  : Node("directed_map_broadcaster")
  {
    RCLCPP_INFO(this->get_logger(), "\033[32mBroadcasting Directed Map\033[0m");

    static_tf_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    for (const auto& point : map_config_["relations"]) {
      std::string pointName = point.as<std::string>();
      std::string parentFrame = pointName.substr(0, pointName.find('/'));
      std::string childFrame = pointName.substr(pointName.find('/') + 1, pointName.length());

      this->broadcast_map(
        parentFrame, // Parent frame
        childFrame, // Child frame
        map_config_[childFrame][0].as<float>() - map_config_[parentFrame][0].as<float>(), // Child x coordinate
        map_config_[childFrame][1].as<float>() - map_config_[parentFrame][1].as<float>() // Child y coordinate
      );
    }
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  YAML::Node map_config_ = YAML::LoadFile(
    ament_index_cpp::get_package_share_directory("mecabot_gz")
    + "/config/directed_map.yaml"
  );

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