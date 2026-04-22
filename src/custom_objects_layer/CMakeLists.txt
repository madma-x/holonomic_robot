cmake_minimum_required(VERSION 3.8)
project(custom_objects_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(custom_objects_layer_plugin SHARED
  src/custom_objects_layer.cpp
)

target_include_directories(custom_objects_layer_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(custom_objects_layer_plugin
  nav2_costmap_2d
  pluginlib
  rclcpp
  visualization_msgs)

pluginlib_export_plugin_description_file(nav2_costmap_2d plugin.xml)

install(
  TARGETS custom_objects_layer_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
