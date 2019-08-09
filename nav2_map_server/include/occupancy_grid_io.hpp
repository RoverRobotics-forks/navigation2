#include <string>
#include "map_mode.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#pragma once

namespace nav2_map
{
nav_msgs::msg::OccupancyGrid loadMapFromFile(const std::string & path_to_yaml);

void write_map(
  const nav_msgs::msg::OccupancyGrid & map, const std::string & path_no_ext,
  const std::string & image_format, nav2_map::MapMode mode, int8_t free_thresh, int8_t occ_thresh);
}