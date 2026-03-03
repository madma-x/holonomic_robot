cmake_minimum_required(VERSION 3.8)
project(can_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CanFrame.msg"
)

# Build the CAN node executable
add_executable(can_node src/can_node.cpp)

# Build the CAN test node
add_executable(can_test_node src/can_test_node.cpp)

ament_target_dependencies(can_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
)

ament_target_dependencies(can_test_node
  rclcpp
  geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(can_node "${cpp_typesupport_target}")

# Install both executables
install(TARGETS can_node can_test_node DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  rosidl_default_runtime
)

ament_package()