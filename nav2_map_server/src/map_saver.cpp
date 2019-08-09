/*
 * Copyright 2019 Rover Robotics
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
 *     * Neither the name of the <ORGANIZATION> nor the names of its
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

#include "map_saver.hpp"

#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <utility>

#include "Magick++.h"
#include "map_mode.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "occupancy_grid_io.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace nav2_map
{
MapSaver::MapSaver(const rclcpp::NodeOptions & options)
: Node("map_saver", options), save_next_map_promise{}
{
  Magick::InitializeMagick(nullptr);
  {
    mapname_ = declare_parameter("output_file_no_ext", "map");
    if (mapname_.empty()) {
      throw std::runtime_error("Map name not provided");
    }
    threshold_occupied_ = declare_parameter("threshold_occupied", 65);
    if (100 < threshold_occupied_) {
      throw std::runtime_error("Threshold_occupied must be 100 or less");
    }
    threshold_free_ = declare_parameter("threshold_free", 25);
    if (threshold_free_ < 0) {
      throw std::runtime_error("Free threshold must be 0 or greater");
    }
    if (threshold_occupied_ <= threshold_free_) {
      throw std::runtime_error("Threshold_free must be smaller than threshold_occupied");
    }

    std::string mode_str = declare_parameter("map_mode", "trinary");
    try {
      map_mode = map_mode_from_string(mode_str);
    } catch (std::invalid_argument &) {
      map_mode = MapMode::Trinary;
      RCLCPP_WARN(
        get_logger(), "Map mode parameter not recognized: '%s', using default value (trinary)",
        mode_str.c_str());
    }

    image_format = declare_parameter("image_format", map_mode == MapMode::Scale ? "png" : "pgm");
    std::transform(
      image_format.begin(), image_format.end(), image_format.begin(),
      [](unsigned char c) { return std::tolower(c); });
    const std::vector<std::string> BLESSED_FORMATS{"bmp", "pgm", "png"};
    if (
      std::find(BLESSED_FORMATS.begin(), BLESSED_FORMATS.end(), image_format) ==
      BLESSED_FORMATS.end()) {
      std::stringstream ss;
      bool first = true;
      for (auto & format_name : BLESSED_FORMATS) {
        if (!first) {
          ss << ", ";
        }
        ss << "'" << format_name << "'";
        first = false;
      }
      RCLCPP_WARN(
        get_logger(), "Requested image format '%s' is not one of the recommended formats: %s",
        image_format.c_str(), ss.str().c_str());
    }
    const std::string FALLBACK_FORMAT = "png";

    try {
      Magick::CoderInfo info(image_format);
      if (!info.isWritable()) {
        RCLCPP_WARN(
          get_logger(), "Format '%s' is not writable. Using '%s' instead", image_format.c_str(),
          FALLBACK_FORMAT.c_str());
        image_format = FALLBACK_FORMAT;
      }
    } catch (Magick::ErrorOption & e) {
      RCLCPP_WARN(
        get_logger(), "Format '%s' is not usable. Using '%s' instead:\n%s", image_format.c_str(),
        FALLBACK_FORMAT.c_str(), e.what());
      image_format = FALLBACK_FORMAT;
    }
    if (
      map_mode == MapMode::Scale &&
      (image_format == "pgm" || image_format == "jpg" || image_format == "jpeg")) {
      RCLCPP_WARN(
        get_logger(),
        "Map mode 'scale' requires transparency, but format '%s' does not support it. Consider "
        "switching image format to 'png'.",
        image_format.c_str());
    }

    RCLCPP_INFO(get_logger(), "Waiting for the map");
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::SystemDefaultsQoS(),
      std::bind(&MapSaver::mapCallback, this, std::placeholders::_1));
  }
}

void MapSaver::try_write_map_to_file(const nav_msgs::msg::OccupancyGrid & map)
{
  RCLCPP_INFO(
    get_logger(), "Received a %d X %d map @ %.3f m/pix", map.info.width, map.info.height,
    map.info.resolution);
  RCLCPP_INFO(get_logger(), "Writing to file");
  write_map(map, mapname_, image_format, map_mode, threshold_free_, threshold_occupied_);
  RCLCPP_INFO(get_logger(), "Map saved");
}

void MapSaver::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
  auto current_promise = std::move(save_next_map_promise);
  save_next_map_promise = std::promise<void>();

  try {
    try_write_map_to_file(*map);
    current_promise.set_value();
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to write map for reason: %s", e.what());
    current_promise.set_exception(std::current_exception());
  }
}
}  // namespace nav2_map
