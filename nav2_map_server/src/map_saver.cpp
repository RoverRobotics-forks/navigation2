// Copyright 2019 Rover Robotics
// Copyright 2008, Willow Garage, Inc.
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

#include "nav2_map_server/map_saver.hpp"

#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <utility>

#include "Magick++.h"
#include "nav2_map_server/map_mode.hpp"
#include "nav_msgs/msg/occupancy_grid.h"
#include "nav_msgs/srv/get_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "yaml-cpp/yaml.h"

namespace nav2_map_server
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

    image_format = declare_parameter("image_format", "pgm");

    RCLCPP_INFO(get_logger(), "Waiting for the map");
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::SystemDefaultsQoS(),
      std::bind(&MapSaver::mapCallback, this, std::placeholders::_1));
  }
}

void MapSaver::try_write_map_to_file(const nav_msgs::msg::OccupancyGrid & map)
{
  rclcpp::Logger logger = this->get_logger();
  RCLCPP_INFO(
    logger, "Received a %d X %d map @ %.3f m/pix", map.info.width, map.info.height,
    map.info.resolution);

  Magick::CoderInfo info(image_format);
  if (!info.isWritable()) {
    image_format = "pgm";
    RCLCPP_WARN(logger, "Format is not writable. Falling back to %s", image_format.c_str());
  }

  std::string mapdatafile = mapname_ + "." + image_format;
  {
    Magick::Geometry size{map.info.width, map.info.height};
    Magick::Color color{"red"};
    Magick::Image image(size, color);
    // Since we only need to support 100 different pixel levels, 8 bits is fine
    image.depth(8);
    image.magick(image_format);
    if (
      map_mode == MapMode::Scale &&
      (image_format == "pgm" || image_format == "jpg" || image_format == "jpeg"))
    {
      RCLCPP_WARN(
        logger,
        "Map mode 'scale' requires transparency, but format '%s' does not support it. Consider "
        "switching image format to 'png'.",
        image_format.c_str());
    }
    for (size_t y = 0; y < map.info.height; y++) {
      for (size_t x = 0; x < map.info.width; x++) {
        size_t i = x + (map.info.height - y - 1) * map.info.width;
        int8_t map_cell = map.data[i];

        Magick::Color pixel;

        switch (map_mode) {
          case MapMode::Trinary:
            if (map_cell == -1) {
              pixel = Magick::ColorGray(205 / 255.0);
            } else if (map_cell <= threshold_free_) {
              pixel = Magick::ColorGray(254 / 255.0);
            } else if (map_cell >= threshold_occupied_) {
              pixel = Magick::ColorGray(0 / 255.0);
            } else {
              pixel = Magick::ColorGray(205 / 255.0);
            }
            break;
          case MapMode::Scale:
            if (map_cell == -1) {
              pixel = Magick::Color{};
            } else {
              pixel = Magick::ColorGray((100.0 - map_cell) / 100.0);
            }
            break;
          case MapMode::Raw:
            if (map_cell == -1) {
              pixel = Magick::ColorGray(1.0);
            } else {
              pixel = Magick::ColorGray(map_cell * 256.0 / 100.0);
            }
            break;
          default:
            throw std::runtime_error("Invalid map mode");
        }
        image.pixelColor(x, y, pixel);
      }
    }
    RCLCPP_INFO(logger, "Writing map occupancy data to %s", mapdatafile.c_str());
    image.write(mapdatafile);
  }

  std::string mapmetadatafile = mapname_ + ".yaml";
  {
    std::ofstream yaml(mapmetadatafile);

    geometry_msgs::msg::Quaternion orientation = map.info.origin.orientation;
    tf2::Matrix3x3 mat(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    double yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);

    YAML::Emitter e;
    e << YAML::Precision(3);
    e << YAML::BeginMap;
    e << YAML::Key << "image" << YAML::Value << mapdatafile;
    e << YAML::Key << "mode" << YAML::Value << map_mode_to_string(map_mode);
    e << YAML::Key << "resolution" << YAML::Value << map.info.resolution;
    e << YAML::Key << "origin" << YAML::Flow << YAML::BeginSeq << map.info.origin.position.x <<
      map.info.origin.position.y << yaw << YAML::EndSeq;
    e << YAML::Key << "negate" << YAML::Value << 0;
    e << YAML::Key << "occupied_thresh" << YAML::Value << threshold_occupied_ / 100.0;
    e << YAML::Key << "free_thresh" << YAML::Value << threshold_free_ / 100.0;
    e << YAML::Key << YAML::Key;

    if (!e.good()) {
      RCLCPP_WARN(
        logger, "YAML writer failed with an error %s. The map metadata may be invalid.",
        e.GetLastError().c_str());
    }

    RCLCPP_INFO(logger, "Writing map metadata to %s", mapmetadatafile.c_str());
    std::ofstream(mapmetadatafile) << e.c_str();
  }
  RCLCPP_INFO(logger, "Map saved");
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
}  // namespace nav2_map_server
