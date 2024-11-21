/*
  
*/

// C++ std libs
#include <algorithm>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <rclcpp/service.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// RCLCPP utilities
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

// Messages
#include "std_msgs/msg/string.hpp"

// YAML processor library
#include "yaml-cpp/yaml.h"

class PathPlanner : public rclcpp::Node 
{
public:
  PathPlanner() : Node("path_planner"){
    RCLCPP_INFO(this->get_logger(), "\033[32mStarting Path Planner\033[0m");
    
    command_listener_ = this->create_subscription
      <std_msgs::msg::String>("/path_command", 10, 
      [this](std_msgs::msg::String msg) -> void {

        // Parsing command as start and end frames as integer for best practice
        size_t spacePos = msg.data.find(' ');
        if (spacePos != std::string::npos) {
            start_frame_ = msg.data.substr(0, spacePos);
            goal_frame_ = msg.data.substr(spacePos + 1);

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

        /// A* ALGORITHM IMPLEMENTATION ///

        if (!start_frame_.empty() && !goal_frame_.empty()) {
          RCLCPP_INFO(this->get_logger(), "\033[43m\033[37m   Running A*...   \033[0m");
          std::unordered_map<std::string, FramesBuffer> edge_frames;
          std::unordered_map<std::string, FramesBuffer> explored_frames;
          shortest_path_ = {};

          std::string current_frame = start_frame_;
          bool found_path = false;

          while (!found_path) {

            // Get current frame coordinates and convert to integer
            std::vector<int> current_coords = {
              static_cast<int>(floor(map_config_[current_frame][0].as<float>() * 10)),
              static_cast<int>(floor(map_config_[current_frame][1].as<float>() * 10))
            };

            // Find neighboring frames of the current frame then calculate the costs and update the edge_frames map
            for (const auto& joint : map_config_["relations"]) {
              
              // Check if the relation string have the current frame name and has not already been explored
              if (joint.as<std::string>().find(current_frame) != std::string::npos) {
                
                // Parse neighboring frame name
                std::string jointPair = joint.as<std::string>();
                size_t current_frame_pos = jointPair.find(current_frame);

                std::string neighbor = (current_frame_pos > jointPair.find('/')) ? 
                jointPair.substr(0, jointPair.find('/')) : jointPair.substr(jointPair.find('/') + 1);

                // Check if the neighboring frame has not already been explored
                if (!explored_frames.count(neighbor)) {

                  // Parse neighbor coordinate
                  std::vector<int> neighbor_coords = {
                    static_cast<int>(floor(map_config_[neighbor][0].as<float>() * 10)),
                    static_cast<int>(floor(map_config_[neighbor][1].as<float>() * 10))
                  };

                  // Update the edge_frames map
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

            // Find an edge frame with minimum cost
            auto minimum_edge = std::min_element(edge_frames.begin(), edge_frames.end(), 
              [](const auto& f1, const auto& f2) {
              
                // If two edge frames have the same f_cost, compare the h_cost instead 
                if (f1.second.f_cost == f2.second.f_cost) {return f1.second.h_cost < f2.second.h_cost;}
                else {return f1.second.f_cost < f2.second.f_cost;}
              
              }
            );

            // Move edge frame to explored
            explored_frames.insert(*minimum_edge);

            // Check if the newly explored frame have H cost of 0, if so then trace back the path and break loop
            if (minimum_edge->second.h_cost == 0) {
              found_path = true;
              bool path_traced = false;
              std::string path_buffer = minimum_edge->first;

              while(!path_traced) {
                shortest_path_.insert(shortest_path_.begin(), path_buffer);
                if (path_buffer != start_frame_) {
                  path_buffer = explored_frames[path_buffer].frame_before;
                } else {path_traced = true;}
              }
              
              // Convert vector to a single string for logging
              std::stringstream ss; ss << "{";
              for (const auto& i : shortest_path_) {
                ss << i;
                if (i != shortest_path_.back()) {ss << ", ";}
              } ss << "}";

              RCLCPP_INFO(this->get_logger(), "\033[42m\033[37m   Found path   \033[0m\nFrames Path: %s", ss.str().c_str());
            } else {current_frame = minimum_edge->first;}

            // Remove explored frame from edge frames
            edge_frames.erase(minimum_edge->first);
          }

          // Path sequence control
          /*
            Listen to motion control feedback while
            feeding it with coordinate commands
          */
        }

      });
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_listener_;

  struct FramesBuffer {
    int g_cost;
    int h_cost;
    int f_cost;
    std::string frame_before;
  };

  std::string start_frame_{}, goal_frame_{};
  std::vector<int> start_coords_{}, goal_coords_{};
  std::vector<std::string> shortest_path_{};
  YAML::Node map_config_ = YAML::LoadFile( // YAML file loader
    ament_index_cpp::get_package_share_directory("mecabot_gz")
    + "/config/directed_map.yaml"
  );

};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlanner>());
  rclcpp::shutdown();
  return 0;
}