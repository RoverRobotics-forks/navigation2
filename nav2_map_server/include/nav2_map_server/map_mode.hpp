#pragma once

#include <string>
#include <vector>
namespace nav2_map_server
{
/// Describes a mapping between colors and map occupancy status
enum class MapMode
{
  /// value >= occ_th - Occupied (100)
  /// value <= free_th - Free (0)
  /// otherwise - Unknown
  Trinary,
  /// alpha < 1.0 - Unknown
  /// value >= occ_th - Occupied (100)
  /// value <= free_th - Free (0)
  /// otherwise - f( (free_th, occ_th) ) = (0, 100)
  /// (linearly map in between values to (0,100)
  Scale,
  /// value = value
  Raw,
};

/// Returns the name of a given MapMode
/// @throw std::invalid_argument if the given value is not a defined map mode
const char * map_mode_to_string(MapMode map_mode);

/// Returns the MapMode corresponding to a given name
/// @throw std::invalid_argument if the name does not name a map mode
MapMode map_mode_from_string(std::string map_mode_name);
}
