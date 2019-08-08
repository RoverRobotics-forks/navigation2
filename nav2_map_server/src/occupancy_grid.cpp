#include "occupancy_grid.hpp"
#include "GraphicsMagick/Magick++.h"
#include "yaml-cpp/yaml.h"
namespace occupancy_grid
{
struct YAMLData
{
  std::string image_file_name;
  double meters_per_pixel;
  std::array<double, 2> origin;
  double yaw;
  double free_thresh;
  double occupied_thresh;
  MapMode mode;
  bool negate;
};

using fn_color_to_occupancy = function<int8_t(std::array<double, 4>)>;
using fn_occupancy_to_color = function<std::array<double, 4>(int8_t)>

  struct ImageOptions
{
  std::string format;
  bool negate;
};

LoadParameters read_yaml(std::istream & yaml_data);

void write_yaml(std::ostream & dest, const LoadParameters & params);

std::int8_t read_image(std::istream & image_data, fn_color_to_occupancy colorToOccupancy);

void write_image(  std::ostream & dest, const OccupancyGrid & map, fn_occupancy_to_color occupancyToColor, std::string image_format);

nav_msgs::msg::OccupancyGrid loadMapFromFile(const std::string path_to_yaml);
void saveMapToFile(const nav_msgs::msg::OccupancyGrid & map, const std::string path_to_yaml);

}