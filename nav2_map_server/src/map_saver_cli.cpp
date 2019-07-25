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

#include <memory>
#include "nav2_map_server/map_saver.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

const std::string USAGE_STRING{
  "Usage: \n"
  "  map_saver -h\n"
  "  map_saver [--occ <threshold_occupied>] [--free <threshold_free>] [--fmt <image_format>] "
  "[--mode trinary/scale/raw] [-f <mapname>] [ROS remapping args]"};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto logger = rclcpp::get_logger("map_saver_cli");

  std::vector<std::string> arguments(argv + 1, argv + argc);
  std::vector<rclcpp::Parameter> params_from_args;
  for (auto it = arguments.begin(); it != arguments.end(); it++) {
    if (*it == "-h") {
      std::cout << USAGE_STRING << std::endl;
      return 0;
    } else if (*it == "-f") {
      if (++it == arguments.end()) {
        RCLCPP_WARN(logger, "Argument ignored: -f should be followed by a value.");
        continue;
      }
      params_from_args.emplace_back("output_file_no_ext", *it);
    } else if (*it == "--occ") {
      if (++it == arguments.end()) {
        RCLCPP_WARN(logger, "Argument ignored: --occ should be followed by a value.");
        continue;
      }
      params_from_args.emplace_back("threshold_occupied", atoi(it->c_str()));
    } else if (*it == "--free") {
      if (++it == arguments.end()) {
        RCLCPP_WARN(logger, "Argument ignored: --free should be followed by a value.");
        continue;
      }
      params_from_args.emplace_back("threshold_free", atoi(it->c_str()));
    } else if (*it == "--mode") {
      if (++it == arguments.end()) {
        RCLCPP_WARN(logger, "Argument ignored: --mode should be followed by a value.");
        continue;
      }
      params_from_args.emplace_back("map_mode", *it);
    } else if (*it == "--fmt") {
      if (++it == arguments.end()) {
        RCLCPP_WARN(logger, "Argument ignored: --fmt should be followed by a value.");
        continue;
      }
      params_from_args.emplace_back("image_format", *it);
    } else {
      RCLCPP_WARN(logger, "Ignoring unrecognized argument '%s'", it->c_str());
    }
  }

  auto node = std::make_shared<nav2_map_server::MapSaver>(
    rclcpp::NodeOptions().parameter_overrides(params_from_args));
  auto future = node->map_saved_future();
  int retcode;

  rclcpp::spin_until_future_complete(node, future);

  try {
    future.get();
    std::cout << "Map saver succeeded" << std::endl;
    retcode = 0;
  } catch (std::exception & e) {
    std::cout << "Map saver failed" << std::endl;
    retcode = 1;
  }
  rclcpp::shutdown();
  return retcode;
}
