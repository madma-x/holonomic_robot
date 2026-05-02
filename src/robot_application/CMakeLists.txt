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
  scripts/task_planner.py
  scripts/run_arm_sequence.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

# Install behavior tree XML files
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees")
  install(DIRECTORY behavior_trees/
    DESTINATION share/${PROJECT_NAME}/behavior_trees)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
