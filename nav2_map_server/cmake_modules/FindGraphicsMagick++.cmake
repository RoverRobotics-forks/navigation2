# Copyright 2019 Rover Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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