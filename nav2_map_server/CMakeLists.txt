cmake_minimum_required(VERSION 3.5)
project(nav2_map)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_util REQUIRED)
find_package(GRAPHICSMAGICKCPP REQUIRED)

nav2_package()

set(map_server_executable map_server)
set(map_saver_executable map_saver)

set(library_name ${map_server_executable}_core)

include_directories(include)
add_library(${library_name} SHARED
  src/occupancy_grid_io.cpp
  src/map_mode.cpp
  )

target_include_directories(${library_name} SYSTEM PRIVATE ${GRAPHICSMAGICKCPP_INCLUDE_DIRS})
target_link_libraries(${library_name} ${GRAPHICSMAGICKCPP_LIBRARIES})
ament_target_dependencies(${library_name}
  yaml_cpp_vendor
  nav_msgs
  std_msgs)

add_executable(${map_server_executable}
  src/map_server_main.cpp
  src/map_server.cpp)
target_link_libraries(${map_server_executable} ${library_name})
ament_target_dependencies(${map_server_executable}
  rclcpp
  rclcpp_lifecycle
  tf2
  nav2_util
  )

add_executable(${map_saver_executable}
  src/map_saver_cli.cpp
  src/map_saver.cpp)
target_include_directories(${map_saver_executable} SYSTEM PRIVATE ${GRAPHICSMAGICKCPP_INCLUDE_DIRS})
target_link_libraries(${map_saver_executable} ${library_name})
ament_target_dependencies(${map_saver_executable}
  rclcpp
  nav_msgs
  tf2)

install(TARGETS ${map_server_executable} ${library_name} ${map_saver_executable}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_occ_grid test/unit/test_occ_grid.cpp)
  target_link_libraries(test_occ_grid ${library_name})
  target_include_directories(test_occ_grid PRIVATE test)
  ament_target_dependencies(test_occ_grid
    rclcpp nav_msgs)

  add_executable(test_occ_grid_node test/component/test_occ_grid_node.cpp)
  target_link_libraries(test_occ_grid_node ${library_name})
  target_include_directories(test_occ_grid_node PRIVATE test)
  ament_target_dependencies(test_occ_grid_node
    rclcpp nav_msgs)

  add_launch_test(
    test/component/test_occ_grid_launch.py
  )
  add_launch_test(
    test/test_launch_files/map_server_node.launch.py
  )
endif()

ament_export_include_directories(include)
ament_package()
