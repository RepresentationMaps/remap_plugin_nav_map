cmake_minimum_required(VERSION 3.8)
project(remap_plugin_nav_map)

add_compile_options(-fPIC)

# Global flags
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH "${FIND_OPENVDB_PATH}")

find_package(ament_cmake_auto REQUIRED)
find_package(yaml-cpp REQUIRED)
ament_auto_find_build_dependencies()

# Targets
add_library(remap_plugin_nav_map SHARED
src/remap_plugin_nav_map/plugin_nav_map.cpp)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

pluginlib_export_plugin_description_file(remap_plugin_base plugins.xml)

target_include_directories(remap_plugin_nav_map PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  remap_plugin_nav_map
  ament_index_cpp
  geometry_msgs
  nav_msgs
  pluginlib
  rclcpp
  remap_map_handler
  remap_plugin_base
  remap_regions_register
  tf2
  tf2_ros
)

target_link_libraries(
  remap_plugin_nav_map
  yaml-cpp
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  remap_plugin_nav_map
  ament_index_cpp
  geometry_msgs
  yaml-cpp
)

ament_export_targets(
  export_${PROJECT_NAME}
)

install(
  TARGETS remap_plugin_nav_map
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Testing
if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

ament_index_register_resource("remap.plugin_configuration" CONTENT "maps/small_house.yaml")
ament_index_register_resource("pal_configuration.remap_manager" CONTENT config/00-nav_map.yml)

ament_auto_package(INSTALL_TO_SHARE config)
