/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// This file contains helper functions for loading images as maps.
// Author: Brian Gerkey

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
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::string & yaml_filename)
: node_(node), yaml_filename_(yaml_filename)
{
  RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Creating");
}

OccGridLoader::~OccGridLoader()
{
  RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Destroying");
}

nav2_util::CallbackReturn
OccGridLoader::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Configuring");

  msg_ = std::make_unique<nav_msgs::msg::OccupancyGrid>();

  // The YAML document from which to get the conversion parameters
  YAML::Node doc = YAML::LoadFile(yaml_filename_);
  LoadParameters loadParameters;

  // Get the name of the map file
  std::string map_filename;
  try {
    map_filename = doc["image"].as<std::string>();
    if (map_filename.size() == 0) {
      throw std::runtime_error("The image tag cannot be an empty string");
    }
    if (map_filename[0] != '/') {
      // dirname can modify what you pass it
      char * fname_copy = strdup(yaml_filename_.c_str());
      map_filename = std::string(dirname(fname_copy)) + '/' + map_filename;
      free(fname_copy);
    }
  } catch (YAML::Exception & e) {
    std::string err("'" + yaml_filename_ +
      "' does not contain an image tag or it is invalid: " + e.what());
    throw std::runtime_error(err);
  }

  try {
    loadParameters.resolution = doc["resolution"].as<double>();
  } catch (YAML::Exception & e) {
    std::string err("The map does not contain a resolution tag or it is invalid: %s", e.what());
    throw std::runtime_error(err);
  }

  try {
    loadParameters.origin[0] = doc["origin"][0].as<double>();
    loadParameters.origin[1] = doc["origin"][1].as<double>();
    loadParameters.origin[2] = doc["origin"][2].as<double>();
  } catch (YAML::Exception & e) {
    std::string err("The map does not contain an origin tag or it is invalid: %s", e.what());
    throw std::runtime_error(err);
  }

  try {
    loadParameters.free_thresh = doc["free_thresh"].as<double>();
  } catch (YAML::Exception & e) {
    std::string err("The map does not contain a free_thresh tag or it is invalid: %s", e.what());
    throw std::runtime_error(err);
  }

  try {
    loadParameters.occupied_thresh = doc["occupied_thresh"].as<double>();
  } catch (YAML::Exception & e) {
    std::string err("The map does not contain an "
      "occupied_thresh tag or it is invalid: %s", e.what());
    throw std::runtime_error(err);
  }

  std::string mode_str;
  try {
    mode_str = doc["mode"].as<std::string>();

    // Convert the string version of the mode name to one of the enumeration values
    try {
      loadParameters.mode = map_mode_from_string(mode_str);
    } catch (std::invalid_argument &) {
      RCLCPP_WARN(
        node_->get_logger(), "Mode parameter not recognized: '%s', using default value (trinary)",
        mode_str.c_str());
      loadParameters.mode = MapMode::Trinary;
    }
  } catch (YAML::Exception & e) {
    RCLCPP_WARN(node_->get_logger(),
      "Mode parameter not set, using default value (trinary): %s", e.what());
    loadParameters.mode = MapMode::Trinary;
  }

  try {
    loadParameters.negate = doc["negate"].as<int>();
  } catch (YAML::Exception & e) {
    std::string err("The map does not contain a negate tag or it is invalid: %s", e.what());
    throw std::runtime_error(err);
  }

  RCLCPP_DEBUG(node_->get_logger(), "resolution: %f", loadParameters.resolution);
  RCLCPP_DEBUG(node_->get_logger(), "origin[0]: %f", loadParameters.origin[0]);
  RCLCPP_DEBUG(node_->get_logger(), "origin[1]: %f", loadParameters.origin[1]);
  RCLCPP_DEBUG(node_->get_logger(), "origin[2]: %f", loadParameters.origin[2]);
  RCLCPP_DEBUG(node_->get_logger(), "free_thresh: %f", loadParameters.free_thresh);
  RCLCPP_DEBUG(node_->get_logger(), "occupied_thresh: %f", loadParameters.occupied_thresh);
  RCLCPP_DEBUG(node_->get_logger(), "mode_str: %s", mode_str.c_str());
  RCLCPP_DEBUG(node_->get_logger(), "mode: %d", loadParameters.mode);
  RCLCPP_DEBUG(node_->get_logger(), "negate: %d", loadParameters.negate);

  loadMapFromFile(map_filename, loadParameters);

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

nav2_util::CallbackReturn
OccGridLoader::on_activate(const rclcpp_lifecycle::State & /*state*/)
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

nav2_util::CallbackReturn
OccGridLoader::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Deactivating");

  occ_pub_->on_deactivate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
OccGridLoader::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(node_->get_logger(), "OccGridLoader: Cleaning up");

  occ_pub_.reset();
  occ_service_.reset();
  msg_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

void OccGridLoader::loadMapFromFile(
  const std::string & map_name, const LoadParameters & loadParameters)
{
  Magick::InitializeMagick(nullptr);
  nav_msgs::msg::OccupancyGrid msg;
  Magick::Image img;

  img.read(map_name);

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

  RCLCPP_DEBUG(node_->get_logger(), "Read map %s: %d X %d map @ %.3lf m/cell",
    map_name.c_str(),
    msg.info.width, msg.info.height, msg.info.resolution);
  *msg_ = msg;
}

}  // namespace nav2_map_server
