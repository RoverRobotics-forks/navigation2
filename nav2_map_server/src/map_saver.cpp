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

#include "nav2_map_server/map_saver.hpp"
#include <fstream>
#include "Magick++.h"

#include <cstdio>
#include <memory>
#include <string>
#include "nav_msgs/msg/occupancy_grid.h"
#include "nav_msgs/srv/get_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

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
    if (mode_str == "trinary") {
      map_mode = TRINARY;
    } else if (mode_str == "scale") {
      map_mode = SCALE;
    } else if (mode_str == "raw") {
      map_mode = RAW;
    } else {
      map_mode = TRINARY;
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
    RCLCPP_INFO(logger, "mode %d", map_mode);
    if (!image.matte() && map_mode == SCALE) {
      RCLCPP_WARN(
        logger,
        "Map mode 'scale' requires transparency, but format '%s' does not support it. Consider "
        "switching image format to 'PNG'.",
        image_format.c_str());
    }
    for (size_t y = 0; y < map.info.height; y++) {
      for (size_t x = 0; x < map.info.width; x++) {
        size_t i = x + (map.info.height - y - 1) * map.info.width;
        int8_t map_cell = map.data[i];

        Magick::Color pixel;

        switch (map_mode) {
          case TRINARY:
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
          case SCALE:
            if (map_cell == -1) {
              pixel = Magick::Color{};
            } else {
              pixel = Magick::ColorGray((100.0 - map_cell) / 100.0);
            }
            break;
          case RAW:
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
  RCLCPP_INFO(logger, "Writing map metadata to %s", mapmetadatafile.c_str());
  {
    std::ofstream yaml(mapmetadatafile);

    geometry_msgs::msg::Quaternion orientation = map.info.origin.orientation;
    tf2::Matrix3x3 mat(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    double yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);

    yaml << "image: " << mapdatafile.c_str() << "\nresolution: " << map.info.resolution
         << "\norigin: [" << map.info.origin.position.x << ", " << map.info.origin.position.y
         << ", " << yaw << "]\n"
         << "\nnegate: 0"
         << "occupied_thresh: " << threshold_occupied_ / 100.0
         << "free_thresh: " << threshold_free_ / 100.0;
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
