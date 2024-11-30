/*
  
  This node finds shortest path with directed map
  (map of uniquely named nodes) as reference.
  Then runs the robot along the path.

  Accepts custom formatted string message.

*/

#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "std_msgs/msg/string.hpp"

#include "yaml-cpp/yaml.h"

class DirectedPathfinder : public rclcpp::Node 
{
public:
  DirectedPathfinder() : Node("directed_pathfinder") {
    RCLCPP_INFO(this->get_logger(), "\033[32mStarting Directed Pathfinder\033[0m");

    params_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "trajectory_master");
    if (!params_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Trajectory control not available!"); return;}
    
    auto params_future = params_client_->get_parameters({
      "msg_on_idle", "msg_on_run", "msg_on_done"});
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), params_future);

    if (params_future.valid()) {
      idle_msg = params_future.get().at(0).as_string();
      run_msg = params_future.get().at(1).as_string();
      done_msg = params_future.get().at(2).as_string();
    } else {
      RCLCPP_ERROR(this->get_logger(), 
      "Failed to get status message parameters from trajectory control, using defaults");
    }

    trajectory_publisher_= this->create_publisher<std_msgs::msg::String>("/trajectory_command", 10);
    command_listener_ = this->create_subscription<std_msgs::msg::String>("/path_command", 10, 
      std::bind(&DirectedPathfinder::pathSolverCallback_, this, std::placeholders::_1));

    trajectory_status_listener_ = this->create_subscription<std_msgs::msg::String>("/motion_status", 10, 
      std::bind(&DirectedPathfinder::trajectoryExecutor_, this, std::placeholders::_1));
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_listener_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trajectory_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trajectory_status_listener_;
  std::shared_ptr<rclcpp::AsyncParametersClient> params_client_;

  bool move_to_path{false};
  std::string idle_msg{"idle"}, run_msg{"in_motion"}, done_msg{"arrived"};

  struct FramesBuffer {
    int g_cost;
    int h_cost;
    int f_cost;
    std::string frame_before;
  };

  std::string start_frame_{}, goal_frame_{};
  std::vector<int> start_coords_{}, goal_coords_{};
  std::vector<std::string> shortest_path_{};
  YAML::Node map_config_ = YAML::LoadFile(
    ament_index_cpp::get_package_share_directory("mecabot_gz")
    + "/config/directed_map.yaml"
  );

  void pathSolverCallback_(std_msgs::msg::String::UniquePtr);
  void trajectoryExecutor_(std_msgs::msg::String::UniquePtr);
};

void DirectedPathfinder::pathSolverCallback_(std_msgs::msg::String::UniquePtr msg)
{
  // parsing start and end coords as integer for best practice
  size_t spacePos = msg->data.find(' ');
  if (spacePos != std::string::npos) {
      start_frame_ = msg->data.substr(0, spacePos);
      goal_frame_ = msg->data.substr(spacePos + 1);
      start_coords_ = {
        static_cast<int>(floor(map_config_[start_frame_][0].as<float>() * 10)), 
        static_cast<int>(floor(map_config_[start_frame_][1].as<float>() * 10))
      };
      goal_coords_ = {
        static_cast<int>(floor(map_config_[goal_frame_][0].as<float>() * 10)),
        static_cast<int>(floor(map_config_[goal_frame_][1].as<float>() * 10))
      };
  } else {
      start_frame_ = "";
      goal_frame_ = "";
      RCLCPP_INFO(this->get_logger(), "Command invalid, no whitespace");
  }

  // A* starts here
  if (!start_frame_.empty() && !goal_frame_.empty()) {
    RCLCPP_INFO(this->get_logger(), "\033[43m\033[37m   Running A*...   \033[0m");
    
    std::unordered_map<std::string, FramesBuffer> edge_frames;
    std::unordered_map<std::string, FramesBuffer> explored_frames;
    
    bool found_path = false;
    shortest_path_ = {};
    std::string current_frame = start_frame_;
    
    while (!found_path) {
      std::vector<int> current_coords = {
        static_cast<int>(floor(map_config_[current_frame][0].as<float>() * 10)),
        static_cast<int>(floor(map_config_[current_frame][1].as<float>() * 10))
      };

      // discover neighbors of the current working frame
      for (const auto& joint : map_config_["relations"]) {

        if (joint.as<std::string>().find(current_frame) != std::string::npos) {
          std::string jointPair = joint.as<std::string>();
          size_t current_frame_pos = jointPair.find(current_frame);

          std::string neighbor = (current_frame_pos > jointPair.find('/')) ? 
          jointPair.substr(0, jointPair.find('/')) : jointPair.substr(jointPair.find('/') + 1);

          if (!explored_frames.count(neighbor)) {
            std::vector<int> neighbor_coords = {
              static_cast<int>(floor(map_config_[neighbor][0].as<float>() * 10)),
              static_cast<int>(floor(map_config_[neighbor][1].as<float>() * 10))
            };

            edge_frames[neighbor].h_cost = static_cast<int>(floor(hypot(
              goal_coords_[0] - neighbor_coords[0], goal_coords_[1] - neighbor_coords[1])));

            edge_frames[neighbor].g_cost = static_cast<int>(floor(hypot(
              neighbor_coords[0] - current_coords[0], neighbor_coords[1] - current_coords[1])))
              + ((current_frame == start_frame_) ? 0 : explored_frames[current_frame].g_cost);

            edge_frames[neighbor].f_cost = edge_frames[neighbor].h_cost + edge_frames[neighbor].g_cost;
            edge_frames[neighbor].frame_before = current_frame;
          }
        }
      }

      // get minimum edge frame
      auto minimum_edge = std::min_element(edge_frames.begin(), edge_frames.end(), 
        [](const auto& f1, const auto& f2) {
          if (f1.second.f_cost == f2.second.f_cost) {return f1.second.h_cost < f2.second.h_cost;}
          else {return f1.second.f_cost < f2.second.f_cost;}
      }); explored_frames.insert(*minimum_edge);

      // trace back path from the goal frame to start
      if (minimum_edge->second.h_cost == 0) {
        std::string path_buffer = minimum_edge->first;
        found_path = true;
        bool path_traced = false;

        while(!path_traced) {
          shortest_path_.insert(shortest_path_.begin(), path_buffer);
          
          if (path_buffer != start_frame_) {
            path_buffer = explored_frames[path_buffer].frame_before;
          } else {
            path_traced = true;
          }
        }
        
        // convert vector to a single string for logging
        std::stringstream ss; ss << "{";
        for (const auto& i : shortest_path_) {
          ss << i;
          if (i != shortest_path_.back()) {ss << ", ";}
        } ss << "}";
        RCLCPP_INFO(this->get_logger(), "\033[42m\033[37m   Found path   \033[0m\nFrames Path: %s", ss.str().c_str());

      } else {current_frame = minimum_edge->first;}

      edge_frames.erase(minimum_edge->first); // exclude newly explored frame from edges
    }
    
    // initialize path motion
    shortest_path_.erase(shortest_path_.begin()); 

    std_msgs::msg::String msg;
    msg.data = "point/abs/"
    + std::to_string(map_config_[shortest_path_.at(0)][0].as<float>()) 
    + ',' + std::to_string(map_config_[shortest_path_.at(0)][1].as<float>())
    + ',' + std::to_string(map_config_[shortest_path_.at(0)][2].as<float>());

    trajectory_publisher_->publish(msg);
    move_to_path = true;
    RCLCPP_INFO(this->get_logger(), "\033[43m\033[37m   Traversing path...   \033[0m");
  }
}

void DirectedPathfinder::trajectoryExecutor_(std_msgs::msg::String::UniquePtr status)
{
  if (status->data == done_msg && move_to_path) {
    shortest_path_.erase(shortest_path_.begin());

    if (!shortest_path_.empty()) {
      std_msgs::msg::String msg;
      msg.data = "point/abs/"
      + std::to_string(map_config_[shortest_path_.at(0)][0].as<float>()) 
      + ',' + std::to_string(map_config_[shortest_path_.at(0)][1].as<float>())
      + ',' + std::to_string(map_config_[shortest_path_.at(0)][2].as<float>());

      trajectory_publisher_->publish(msg);
    } else {
      move_to_path = false;
      RCLCPP_INFO(this->get_logger(), "\033[42m\033[37m   Path complete   \033[0m");
    }
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectedPathfinder>());
  rclcpp::shutdown();
  return 0;
}