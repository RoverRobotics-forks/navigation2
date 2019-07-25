# CMake script for finding GraphicsMagick++, the C++ interface for the GraphicsMagick library
#
# Output variables:
#  MAGICK++_FOUND        - system has GraphicsMagick++
#  MAGICK++_INCLUDE_DIRS - include directories for GraphicsMagick
#  MAGICK++_LIBRARIES    - libraries you need to link to

find_path(MAGICK++_INCLUDE_DIR
  NAMES "Magick++.h"
  PATHS ${GraphicsMagick++_INCLUDE_PATH}
  )

find_library(MAGICK++_LIBRARY
  NAMES GraphicsMagick++
  PATHS ${GraphicsMagick++_LIBRARY_PATH}
  )

set(MAGICK++_LIBRARIES ${MAGICK++_LIBRARY})
set(MAGICK++_INCLUDE_DIRS ${MAGICK++_INCLUDE_DIR})

if(${MAGICK++_INCLUDE_DIR} AND ${MAGICK++_LIBRARIES})
  set(MAGICK++_FOUND YES)
else()
  set(MAGICK++_FOUND NO)
endif()

if(MAGICK++_FOUND)
  # make FIND_PACKAGE friendly
  if(GraphicsMagick++_FIND_REQUIRED)
    message(FATAL_ERROR
      "GraphicsMagick was not found")
  else(Magick_FIND_REQUIRED)
    message(STATUS "GraphicsMagick was not found.")
  endif(GraphicsMagick++_FIND_REQUIRED)
endif(MAGICK++_FOUND)
