cmake_minimum_required(VERSION 3.5)
project(flocking)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add executable
add_executable(flocking_node src/flocking_node.cpp)

# Link dependencies
ament_target_dependencies(flocking_node rclcpp geometry_msgs)

# Install executable
install(TARGETS
  flocking_node
  DESTINATION lib/${PROJECT_NAME})

# Export dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
 #ament_lint_auto_find_test_dependencies()
endif()

ament_package()