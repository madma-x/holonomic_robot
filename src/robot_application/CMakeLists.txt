cmake_minimum_required(VERSION 3.8)
project(robot_application)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/game_state_manager.py
  scripts/mission_executor.py
  scripts/odom_theta_monitor.py
  scripts/safety_arm_reset.py
  scripts/task_planner.py
  scripts/run_arm_sequence.py
  scripts/custom_object_delete_test
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

# Install maps
install(DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps)

# Install images
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/images")
  install(DIRECTORY images/
    DESTINATION share/${PROJECT_NAME}/images)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
