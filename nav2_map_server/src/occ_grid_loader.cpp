// Copyright 2019 Rover Robotics
// Copyright 2018 Brian Gerkey
// Copyright (c) 2008, Willow Garage, Inc.
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

#include "nav2_map_server/occ_grid_loader.hpp"

#include <libgen.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "Magick++.h"
#include "tf2/LinearMath/Quaternion.h"
#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;

namespace nav2_map_server
{
OccGridLoader::OccGridLoader(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string & yaml_filename)
: node_(node), yaml_filename_(yaml_filename)
{
  RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Creating");
}

OccGridLoader::~OccGridLoader() {RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Destroying");}

/// Get the given subnode value.
/// The only reason this function exists is to wrap the exceptions in slightly nicer error messages,
/// including the name of the failed key
template<typename T>
T yaml_get_value(const YAML::Node & node, std::string key)
{
  try {
    return node[key].as<T>();
  } catch (YAML::Exception & e) {
    std::stringstream ss;
    ss << "Failed to parse YAML tag '" << key << "' at line " << e.mark.line << ", column " <<
      e.mark.column << " for reason: " << e.msg;
    throw std::runtime_error(ss.str());
  }
}

OccGridLoader::LoadParameters OccGridLoader::load_map_yaml(const std::string & yaml_filename)
{
  YAML::Node doc = YAML::LoadFile(yaml_filename);
  LoadParameters loadParameters;

  auto image_file_name = doc["image"].as<std::string>();
  if (image_file_name.empty()) {
    throw std::runtime_error("The image tag cannot be an empty string");
  }
  if (image_file_name[0] != '/') {
    // since dirname takes a mutable char *
    std::vector<char> fname_copy(yaml_filename.begin(), yaml_filename.end());
    image_file_name = std::string(dirname(fname_copy.data())) + '/' + image_file_name;
  }
  loadParameters.image_file_name = image_file_name;
  loadParameters.resolution = doc["resolution"].as<double>();
  loadParameters.origin = doc["origin"].as<std::vector<double>>();
  if (loadParameters.origin.size() != 3) {
    throw std::runtime_error(
            "origin should have 3 elements, not " + std::to_string(loadParameters.origin.size()));
  }

  loadParameters.free_thresh = doc["free_thresh"].as<double>();
  loadParameters.occupied_thresh = doc["occupied_thresh"].as<double>();

  auto map_mode_node = doc["mode"];
  if (!map_mode_node.IsDefined()) {
    loadParameters.mode = MapMode::Trinary;
  } else {
    loadParameters.mode = map_mode_from_string(map_mode_node.as<std::string>());
  }

  auto negate_node = doc["negate"];
  try {
    loadParameters.negate = negate_node.as<int>();
  } catch (YAML::BadConversion &) {
    loadParameters.negate = negate_node.as<bool>();
  }

  RCLCPP_DEBUG(node_->get_logger(), "resolution: %f", loadParameters.resolution);
  RCLCPP_DEBUG(node_->get_logger(), "origin[0]: %f", loadParameters.origin[0]);
  RCLCPP_DEBUG(node_->get_logger(), "origin[1]: %f", loadParameters.origin[1]);
  RCLCPP_DEBUG(node_->get_logger(), "origin[2]: %f", loadParameters.origin[2]);
  RCLCPP_DEBUG(node_->get_logger(), "free_thresh: %f", loadParameters.free_thresh);
  RCLCPP_DEBUG(node_->get_logger(), "occupied_thresh: %f", loadParameters.occupied_thresh);
  RCLCPP_DEBUG(node_->get_logger(), "mode: %s", map_mode_to_string(loadParameters.mode));
  RCLCPP_DEBUG(node_->get_logger(), "negate: %d", loadParameters.negate);

  return loadParameters;
}

nav2_util::CallbackReturn OccGridLoader::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Configuring");

  msg_ = std::make_unique<nav_msgs::msg::OccupancyGrid>();
  LoadParameters loadParameters;
  try {
    loadParameters = load_map_yaml(yaml_filename_);
  } catch (YAML::Exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(), "Failed to parse map YAML loaded from file %s for reason: %s",
      yaml_filename_.c_str(), e.what());
    throw;
  }
  loadMapFromFile(loadParameters);

  // Create a service callback handle
  auto handle_occ_callback = [this](
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<nav_msgs::srv::GetMap::Request>/*request*/,
    std::shared_ptr<nav_msgs::srv::GetMap::Response> response) -> void {
      RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Handling map request");
      response->map = *msg_;
    };

  // Create a service that provides the occupancy grid
  occ_service_ = node_->create_service<nav_msgs::srv::GetMap>(service_name_, handle_occ_callback);

  // Create a publisher using the QoS settings to emulate a ROS1 latched topic
  occ_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
    topic_name_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn OccGridLoader::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Activating");

  // Publish the map using the latched topic
  occ_pub_->on_activate();
  occ_pub_->publish(*msg_);

  // due to timing / discovery issues, need to republish map
  auto timer_callback = [this]() -> void {occ_pub_->publish(*msg_);};
  timer_ = node_->create_wall_timer(2s, timer_callback);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn OccGridLoader::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Deactivating");

  occ_pub_->on_deactivate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn OccGridLoader::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Cleaning up");

  occ_pub_.reset();
  occ_service_.reset();
  msg_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

void OccGridLoader::loadMapFromFile(const LoadParameters & loadParameters)
{
  Magick::InitializeMagick(nullptr);
  nav_msgs::msg::OccupancyGrid msg;
  Magick::Image img;

  img.read(loadParameters.image_file_name);

  // todo: do we need img.throwImageException()? here
  /*
  if (!(img = IMG_Load(map_name.c_str()))) {
    std::string errmsg =
      std::string("failed to open image file \"") + map_name + std::string("\": ") + IMG_GetError();
    throw std::runtime_error(errmsg);
  }
   */

  // Copy the image data into the map structure
  msg.info.width = img.size().width();
  msg.info.height = img.size().height();
  msg.info.resolution = loadParameters.resolution;
  msg.info.origin.position.x = loadParameters.origin[0];
  msg.info.origin.position.y = loadParameters.origin[1];
  msg.info.origin.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, loadParameters.origin[2]);
  msg.info.origin.orientation.x = q.x();
  msg.info.origin.orientation.y = q.y();
  msg.info.origin.orientation.z = q.z();
  msg.info.origin.orientation.w = q.w();

  // Allocate space to hold the data
  msg.data.resize(msg.info.width * msg.info.height);

  // Copy pixel data into the map structure
  for (size_t j = 0; j < msg.info.height; j++) {
    for (size_t i = 0; i < msg.info.width; i++) {
      auto pixel = img.pixelColor(i, j);

      // shade of the pixel: 0=black 1=white
      auto shade = Magick::Color::scaleQuantumToDouble(
        (pixel.redQuantum() + pixel.blueQuantum() + pixel.greenQuantum()) / 3.0);

      // If negate is true, we consider blacker pixels free, and whiter
      // pixels free.  Otherwise, it's vice versa.
      auto occ = loadParameters.negate ? shade : 1.0 - shade;

      uint8_t map_cell;
      switch (loadParameters.mode) {
        case MapMode::Trinary:
          map_cell =
            occ > loadParameters.occupied_thresh ? +100 : occ < loadParameters.free_thresh ? 0 : -1;
          break;
        case MapMode::Scale:
          map_cell = pixel.alpha() < 1.0 ? -1 : occ * 100;
          break;
        case MapMode::Raw:
          map_cell = (uint8_t)(occ * 256);
          break;
        default:
          throw std::runtime_error("Invalid map mode");
      }
      msg.data[msg.info.width * j + i] = map_cell;
    }
  }

  msg.info.map_load_time = node_->now();
  msg.header.frame_id = frame_id_;
  msg.header.stamp = node_->now();

  RCLCPP_DEBUG(
    node_->get_logger(), "Read map %s: %d X %d map @ %.3lf m/cell",
    loadParameters.image_file_name.c_str(), msg.info.width, msg.info.height, msg.info.resolution);
  *msg_ = msg;
}

}  // namespace nav2_map_server
