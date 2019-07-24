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
const std::string USAGE_STRING{
  "Usage: \n"
  "  map_saver -h\n"
  "  map_saver [--occ <threshold_occupied>] [--free <threshold_free>] "
  "[-f <mapname>] [ROS remapping args]"};

MapSaver::MapSaver(const rclcpp::NodeOptions & options) : Node("map_saver"), save_next_map_promise{}
{
  {
    auto & arguments = options.arguments();
    std::vector<rclcpp::Parameter> params_from_args;
    for (auto it = arguments.begin(); it != arguments.end(); it++) {
      std::cerr << "processing argument " << *it;
      if (*it == "-h") {
        params_from_args.emplace_back("show_help", true);
      } else if (*it == "-f") {
        if (++it == arguments.end()) {
          RCLCPP_WARN(get_logger(), "Argument ignored: -f should be followed by a value.");
          continue;
        }
        params_from_args.emplace_back("output_file_no_ext", *it);
      } else if (*it == "--occ") {
        if (++it == arguments.end()) {
          RCLCPP_WARN(get_logger(), "Argument ignored: --occ should be followed by a value.");
          continue;
        }
        params_from_args.emplace_back("threshold_occupied", atoi(it->c_str()));
      } else if (*it == "--free") {
        if (++it == arguments.end()) {
          RCLCPP_WARN(get_logger(), "Argument ignored: --free should be followed by a value.");
          continue;
        }
        params_from_args.emplace_back("threshold_free", atoi(it->c_str()));
      } else if (*it == "--mode") {
        if (++it == arguments.end()) {
          RCLCPP_WARN(get_logger(), "Argument ignored: --mode should be followed by a value.");
          continue;
        }
        params_from_args.emplace_back("map_mode", *it);
      } else {
        RCLCPP_WARN(get_logger(), "Ignoring unrecognized argument '%s'", it->c_str());
      }
    }

    show_help = declare_parameter("show_help", false);
    if (show_help) {
      std::cout << USAGE_STRING << std::endl;
      return;
    }
    mapname_ = declare_parameter("output_file_no_ext", "map");
    if (mapname_.empty()) {
      throw std::runtime_error("Map name not provided");
    }
    threshold_occupied_ = declare_parameter("threshold_occupied", 65);
    if (threshold_occupied_ < 1 || 100 < threshold_occupied_) {
      throw std::runtime_error("Threshold_occupied must be between 1 and 100");
    }
    threshold_free_ = declare_parameter("threshold_free", 25);
    if (threshold_free_ < 0 || 100 < threshold_free_) {
      throw std::runtime_error("Free threshold must be between 0 and 100");
    }
    if (threshold_occupied_ <= threshold_free_) {
      throw std::runtime_error("Threshold_free must be smaller than threshold_occupied");
    }

    std::string mode_str = declare_parameter("mode", "trinary");
    if (mode_str == "trinary") {
      map_mode = TRINARY;
    } else if (mode_str == "scale") {
      map_mode = SCALE;
    } else if (mode_str == "raw") {
      map_mode = RAW;
    } else {
      RCLCPP_WARN(
        get_logger(), "Mode parameter not recognized: '%s', using default value (trinary)",
        mode_str.c_str());
      map_mode = TRINARY;
    }

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

  std::string mapdatafile = mapname_ + ".pgm";
  RCLCPP_INFO(logger, "Writing map occupancy data to %s", mapdatafile.c_str());
  {
    std::ofstream map_data(mapdatafile);
    map_data.exceptions(~std::ostream::goodbit);

    // map_data.open(mapdatafile.c_str());
    //    fprintf(
    //      map_data, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n", map.info.resolution,
    //      map->info.width, map->info.height);
    map_data << "P5\n"
             << "# CREATOR: map_saver.cpp " << map.info.resolution << " m/pix\n"
             << map.info.width << " " << map.info.height << "\n255\n";
    map_data.exceptions();
    for (size_t y = 0; y < map.info.height; y++) {
      for (size_t x = 0; x < map.info.width; x++) {
        size_t i = x + (map.info.height - y - 1) * map.info.width;
        int8_t raw_pixel = map.data[i];
        switch (map_mode) {
          case TRINARY:
            map_data.put(
              raw_pixel == -1
                ? 205
                : raw_pixel <= threshold_free_ ? 254 : raw_pixel >= threshold_occupied_ ? 0 : 205);
            break;
          case SCALE:
            map_data.put(
              raw_pixel == -1
                ? 205
                : raw_pixel <= threshold_free_ ? 254 : raw_pixel >= threshold_occupied_ ? 0 : 205);
            break;
          case RAW:
            map_data.put(raw_pixel);
            break;
        }
      }
    }
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
