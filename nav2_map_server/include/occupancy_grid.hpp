
#pragma once

namespace nav2_occupancy_grid
{

nav_msgs::msg::OccupancyGrid loadMapFromFile(const std::string path_to_yaml);
void saveMapToFile(const nav_msgs::msg::OccupancyGrid & map, const std::string path_to_yaml);



}