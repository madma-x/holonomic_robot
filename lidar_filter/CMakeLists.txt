cmake_minimum_required(VERSION 3.5)
project(lidar_filter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(scan_filter src/scan_filter.cpp)
ament_target_dependencies(scan_filter
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
)

install(TARGETS scan_filter
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
