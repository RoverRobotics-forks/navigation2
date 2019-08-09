#include "occupancy_grid_io.hpp"
#include <cmath>
#include <fstream>
#include "GraphicsMagick/Magick++.h"
#include "geometry_msgs/msg/pose.hpp"
#include "iostream"
#include "map_mode.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "yaml-cpp/yaml.h"

#include "rcpputils/filesystem_helper.hpp"

using path = rcpputils::fs::path;

using geometry_msgs::msg::Pose;
using nav_msgs::msg::OccupancyGrid;

namespace nav2_map
{
struct MapYAMLData
{
  std::string image_file_name;
  double meters_per_pixel{};
  Pose origin;
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

using fn_color_to_occupancy = std::function<int8_t(Magick::Color)>;
using fn_occupancy_to_color = std::function<Magick::Color(int8_t)>;

Magick::Color occupancy_to_color(
  int8_t map_cell, MapMode map_mode, int8_t threshold_free, int8_t threshold_occupied)
{
  switch (map_mode) {
    case MapMode::Trinary:
      if (map_cell < 0 || 100 < map_cell) {
        return Magick::ColorGray(205 / 255.0);
      } else if (map_cell <= threshold_free) {
        return Magick::ColorGray(254 / 255.0);
      } else if (threshold_occupied <= map_cell) {
        return Magick::ColorGray(0 / 255.0);
      } else {
        return Magick::ColorGray(205 / 255.0);
      }
      break;
    case MapMode::Scale:
      if (map_cell < 0 || 100 < map_cell) {
        auto color = Magick::ColorGray{0.5};
        color.alphaQuantum(TransparentOpacity);
        return color;
      } else {
        return Magick::ColorGray{(100.0 - map_cell) / 100.0};
      }
      break;
    case MapMode::Raw:
      Magick::Quantum q;
      if (map_cell < 0 || 100 < map_cell) {
        q = MaxRGB;
      } else {
        q = map_cell / 255.0 * MaxRGB;
      }
      return Magick::Color(q, q, q);
      break;
    default:
      throw std::runtime_error("Invalid map mode");
  }
}

int8_t color_to_occupancy(
  const Magick::Color & color, MapMode mode, bool negate, double free_thresh,
  double occupied_thresh)
{
  std::vector<Magick::Quantum> channels = {color.redQuantum(), color.greenQuantum(),
                                           color.blueQuantum()};

  double sum = 0;
  for (auto c : channels) {
    sum += c;
  }
  /// on a scale from 0.0 to 1.0 how bright is the pixel?
  double shade = Magick::ColorGray::scaleQuantumToDouble(sum / channels.size());

  // If negate is true, we consider blacker pixels free, and whiter
  // pixels occupied. Otherwise, it's vice versa.
  /// on a scale from 0.0 to 1.0, how occupied is the map cell (before thresholding)?
  double occ = (negate ? shade : 1.0 - shade);

  switch (mode) {
    case MapMode::Trinary:
      if (occupied_thresh < occ) {
        return 100;
      } else if (occ < free_thresh) {
        return 0;
      } else {
        return -1;
      }
      break;
    case MapMode::Scale:
      if (color.alphaQuantum() != OpaqueOpacity) {
        return -1;
      } else if (occupied_thresh < occ) {
        return 100;
      } else if (occ < free_thresh) {
        return 0;
      } else {
        return std::rint((occ - free_thresh) / (occupied_thresh - free_thresh) * 100.0);
      }
      break;
    case MapMode::Raw: {
      double occ_percent = std::round(shade * 255);
      if (0 <= occ_percent && occ_percent <= 100) {
        return static_cast<int8_t>(occ_percent);
      } else {
        return -1;
      }
      break;
    }
    default:
      throw std::runtime_error("Invalid map mode");
  }
}

/// Get the given subnode value.
/// The only reason this function exists is to wrap the exceptions in slightly nicer error messages,
/// including the name of the failed key
/// @throw YAML::Exception
template <typename T>
T yaml_get_value(const YAML::Node & node, const std::string & key)
{
  try {
    return node[key].as<T>();
  } catch (YAML::Exception & e) {
    std::stringstream ss;
    ss << "Failed to parse YAML tag '" << key << "' for reason: " << e.msg;
    throw YAML::Exception(e.mark, ss.str());
  }
}

/// Read map YAML file and return the associated data object
/// @throw YAML::Exception if something goes wrong
MapYAMLData read_map_yaml(std::istream & yaml_data)
{
  YAML::Node doc = YAML::Load(yaml_data);

  MapYAMLData yamlData;

  auto image_file_name = yaml_get_value<std::string>(doc, "image");
  if (image_file_name.empty()) {
    throw YAML::Exception(doc["image"].Mark(), "The image tag was empty.");
  }
  yamlData.image_file_name = image_file_name;

  yamlData.meters_per_pixel = yaml_get_value<double>(doc, "resolution");
  auto origin_x_y_yaw = yaml_get_value<std::array<double, 3>>(doc, "origin");
  yamlData.origin.position.x = origin_x_y_yaw[0];
  yamlData.origin.position.y = origin_x_y_yaw[1];
  yamlData.origin.orientation.w = cos(origin_x_y_yaw[2]);
  yamlData.origin.orientation.z = sin(origin_x_y_yaw[2]);
  yamlData.free_thresh = yaml_get_value<double>(doc, "free_thresh");
  yamlData.occupied_thresh = yaml_get_value<double>(doc, "occupied_thresh");

  auto map_mode_node = doc["mode"];
  if (!map_mode_node.IsDefined()) {
    yamlData.mode = MapMode::Trinary;
  } else {
    yamlData.mode = map_mode_from_string(map_mode_node.as<std::string>());
  }

  try {
    yamlData.negate = yaml_get_value<int>(doc, "negate");
  } catch (YAML::Exception &) {
    yamlData.negate = yaml_get_value<bool>(doc, "negate");
  }

  return yamlData;
};

void write_map_yaml(std::ostream & dest, const MapYAMLData & params)
{
  YAML::Emitter e(dest);
  e << YAML::Precision(3);
  e << YAML::BeginMap;
  e << YAML::Key << "image" << YAML::Value << params.image_file_name;
  e << YAML::Key << "mode" << YAML::Value << map_mode_to_string(params.mode);
  e << YAML::Key << "resolution" << YAML::Value << params.meters_per_pixel;
  e << YAML::Key << "origin" << YAML::Flow << YAML::BeginSeq << params.origin.position.x
    << params.origin.position.y << atan2(params.origin.orientation.z, params.origin.orientation.w)
    << YAML::EndSeq;
  e << YAML::Key << "negate" << YAML::Value << 0;
  e << YAML::Key << "occupied_thresh" << YAML::Value << params.occupied_thresh;
  e << YAML::Key << "free_thresh" << YAML::Value << params.free_thresh;
  if (!e.good()) {
    throw std::runtime_error(
      "YAML writer failed with an error. The map metadata may be invalid:" + e.GetLastError());
  }
};

void write_map_image(
  std::ostream & dest, const MapImageData & map_image_data, const std::string & image_format,
  const fn_occupancy_to_color & occupancyToColor)
{
  Magick::InitializeMagick(nullptr);

  Magick::Blob blob;
  // should never see this color, so the initialization value is just for debugging
  Magick::Image image({map_image_data.width, map_image_data.height}, "red");

  // In scale mode, we need the alpha (matte) channel. Else, we don't.
  // NOTE: GraphicsMagick seems to have trouble loading the alpha channel when saved with
  // Magick::GreyscaleMatte, so we use TrueColorMatte instead.
  image.type(Magick::TrueColorMatteType);

  // Since we only need to support 100 different pixel levels, 8 bits is fine
  image.depth(8);

  for (size_t y = 0; y < map_image_data.height; y++) {
    for (size_t x = 0; x < map_image_data.width; x++) {
      int8_t map_cell =
        map_image_data.data[map_image_data.width * (map_image_data.height - y - 1) + x];
      auto pixel = occupancyToColor(map_cell);
      image.pixelColor(x, y, pixel);
    }
  }
  image.write(&blob, image_format);
  dest.write((char *)blob.data(), blob.length());
}

MapImageData read_map_image(std::istream & src, const fn_color_to_occupancy & colorToOccupancy)
{
  Magick::InitializeMagick(nullptr);
  Magick::Blob blob;

  src.seekg(std::ios::end);
  auto n_bytes = src.tellg();
  src.seekg(std::ios::beg);

  std::vector<char> bytes(n_bytes);

  src.read(bytes.data(), n_bytes);
  blob.updateNoCopy(bytes.data(), n_bytes);

  Magick::Image image(blob);
  MapImageData result;
  result.width = image.size().width();
  result.height = image.size().height();

  // Allocate space to hold the data
  result.data.resize(result.width * result.height);
  // Copy pixel data into the map structure
  for (size_t y = 0; y < result.height; y++) {
    for (size_t x = 0; x < result.width; x++) {
      auto occ = colorToOccupancy(image.pixelColor(x, y));
      result.data[result.width * (result.height - y - 1) + x] = occ;
    }
  }
  return result;
};

OccupancyGrid loadMapFromFile(const std::string & path_to_yaml)
{
  OccupancyGrid result;

  MapYAMLData map_yaml_info;
  try {
    std::ifstream yaml_file(path_to_yaml);
    map_yaml_info = read_map_yaml(yaml_file);
  } catch (YAML::Exception & e) {
    std::stringstream ss;
    ss << "Failed processing YAML file '" << path_to_yaml << "' at position (" << e.mark.line << ":"
       << e.mark.column << ") for reason: " << e.what();
    throw std::runtime_error(ss.str());
  } catch (std::exception & e) {
    std::stringstream ss;
    ss << "Failed processing YAML file '" << path_to_yaml << "' for reason: " << e.what();
    throw std::runtime_error(ss.str());
  }

  result.info.resolution = map_yaml_info.meters_per_pixel;
  result.info.height = -1;
  result.info.width = -1;
  //  result.info.map_load_time = now;  //todo
  result.info.origin = map_yaml_info.origin;

  auto image_path = path(path_to_yaml).parent_path() / map_yaml_info.image_file_name;
  MapImageData image_data;
  try {
    std::ifstream image_fstream(image_path.string(), std::ios::binary);
    image_data = read_map_image(image_fstream, [map_yaml_info](Magick::Color color) {
      return color_to_occupancy(
        color, map_yaml_info.mode, map_yaml_info.negate, map_yaml_info.free_thresh,
        map_yaml_info.occupied_thresh);
    });
  } catch (std::exception & e) {
    std::stringstream ss;
    ss << "Failed processing YAML file '" << path_to_yaml << "' for reason: " << e.what();
  }
  result.info.width = image_data.width;
  result.info.height = image_data.height;
  result.data = std::move(image_data.data);
  return result;
}

void write_map(
  const OccupancyGrid & map, const std::string & path_no_ext, const std::string & image_format,
  MapMode mode, int8_t free_thresh, int8_t occ_thresh)
{
  std::string yaml_ext{".yaml"};
  path image_file_path = path(path_no_ext + "." + image_format);

  MapYAMLData yamlData;
  // we will write the yaml and image file in the same directory, so can omit the path
  yamlData.image_file_name = image_file_path.filename().string();
  yamlData.occupied_thresh = occ_thresh / 100.0;
  yamlData.free_thresh = free_thresh / 100.0;
  yamlData.mode = mode;
  yamlData.meters_per_pixel = map.info.resolution;
  yamlData.origin = map.info.origin;
  std::ofstream yaml_file(path_no_ext + ".yaml");
  write_map_yaml(yaml_file, yamlData);

  std::ofstream image_file(image_file_path.string(), std::ios::binary);
  MapImageData imageData;
  imageData.width = map.info.width;
  imageData.height = map.info.height;
  imageData.data = map.data;
  write_map_image(
    image_file, imageData, image_format, [mode, free_thresh, occ_thresh](int8_t occupancy) {
      return occupancy_to_color(occupancy, mode, free_thresh, occ_thresh);
    });
}

}