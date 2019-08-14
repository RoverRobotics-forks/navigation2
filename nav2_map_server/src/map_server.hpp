// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MAP_SERVER__MAP_SERVER_HPP_
#define NAV2_MAP_SERVER__MAP_SERVER_HPP_

#include <memory>

#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_map
{
class MapServer : public nav2_util::LifecycleNode
{
public:
  MapServer();
  ~MapServer() override;

protected:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map;
  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr srv_map;
  rclcpp::TimerBase::SharedPtr timer;

  // Implement the lifecycle interface
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  // path to YAML file containing map to load
  std::string yaml_filename;

  // The frame ID used in the returned OccupancyGrid message
  std::string frame_id;

  // The name for the topic on which the map will be published
  std::string topic_name;

  // The name of the service for getting a map
  std::string service_name;

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;

  std::chrono::duration<double> map_topic_period{};
};

}  // namespace nav2_map

#endif  // NAV2_MAP_SERVER__MAP_SERVER_HPP_
