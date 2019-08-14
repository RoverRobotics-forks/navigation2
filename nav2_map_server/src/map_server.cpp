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

#include "map_server.hpp"

#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

#include "nav2_util/node_utils.hpp"
#include "occupancy_grid_io.hpp"
#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;

namespace nav2_map
{
MapServer::MapServer() : nav2_util::LifecycleNode("map_server")
{
  RCLCPP_INFO(get_logger(), "Creating");

  declare_parameter("yaml_filename", rclcpp::ParameterValue(std::string("map.yaml")));
}

MapServer::~MapServer() { RCLCPP_INFO(get_logger(), "Destroying"); }

nav2_util::CallbackReturn MapServer::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  frame_id = declare_parameter("frame_id", "map");
  service_name = declare_parameter("service_name", "map");
  topic_name = declare_parameter("topic_name", "map");
  map_topic_period = std::chrono::duration<double>(declare_parameter("topic_period_s", 2.0));

  // Get the name of the YAML file to use
  yaml_filename = declare_parameter("yaml_filename", "map.yaml");

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MapServer::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");

  auto m = std::make_shared<nav_msgs::msg::OccupancyGrid>(loadMapFromFile(yaml_filename));
  m->header.frame_id = frame_id;
  m->header.stamp = now();
  m->info.map_load_time = now();
  map = m;

  pub_map =
    create_publisher<nav_msgs::msg::OccupancyGrid>(topic_name, rclcpp::QoS(1).transient_local());

  srv_map = create_service<nav_msgs::srv::GetMap>(
    service_name, [this](
                    const std::shared_ptr<nav_msgs::srv::GetMap::Request>,
                    std::shared_ptr<nav_msgs::srv::GetMap::Response> response) {
      RCLCPP_INFO(get_logger(), "Handling map request");
      response->map = *map;
    });

  timer = create_wall_timer(map_topic_period, [this]() { pub_map->publish(map); });
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MapServer::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  pub_map.reset();
  timer.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MapServer::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MapServer::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_FATAL(get_logger(), "Lifecycle node entered error state");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MapServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

}  // namespace nav2_map
