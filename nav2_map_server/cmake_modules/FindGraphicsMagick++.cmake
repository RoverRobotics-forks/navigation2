# CMake script for finding GraphicsMagick++, the C++ interface for the GraphicsMagick library
#
# Output variables:
#  GraphicsMagick++_FOUND        - system has GraphicsMagick++
#  GraphicsMagick++_INCLUDE_DIRS - include directories for GraphicsMagick
#  GraphicsMagick++_LIBRARIES    - libraries you need to link to
include(FindPackageHandleStandardArgs)

find_path(GraphicsMagick++_INCLUDE_DIRS
  NAMES "Magick++.h"
  PATH_SUFFIXES GraphicsMagick
  )

find_library(GraphicsMagick++_LIBRARIES
  NAMES "GraphicsMagick++"
  )

find_package_handle_standard_args(
  GraphicsMagick++
  GraphicsMagick++_LIBRARIES
  GraphicsMagick++_INCLUDE_DIRS
)