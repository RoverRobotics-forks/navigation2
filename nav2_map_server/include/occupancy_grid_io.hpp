#include <string>
#include "geometry_msgs/msg/pose.hpp"
#include "map_mode.hpp"
#include "map_mode.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#pragma once

namespace nav2_map
{
struct MapYAMLData
{
  std::string image_file_name;
  double meters_per_pixel{};
  geometry_msgs::msg::Pose origin;
  double free_thresh{};
  double occupied_thresh{};
  MapMode mode{};
  bool negate{};
};

struct MapImageData
{
  std::vector<int8_t> data;
  unsigned width{0};
  unsigned height{0};
};

nav_msgs::msg::OccupancyGrid loadMapFromFile(const std::string & path_to_yaml);

void write_map(
  const nav_msgs::msg::OccupancyGrid & map, const std::string & path_no_ext,
  const std::string & image_format, nav2_map::MapMode mode, int8_t free_thresh, int8_t occ_thresh);
}