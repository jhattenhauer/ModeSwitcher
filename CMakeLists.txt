cmake_minimum_required(VERSION 3.5)
project(waypoint_counter)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)

# Declare your executable
add_executable(waypoint_counter src/tracker.cpp)

# Add dependencies
ament_target_dependencies(waypoint_counter
  rclcpp
  mavros_msgs
)

# Install the executable
install(TARGETS waypoint_counter
  DESTINATION lib/${PROJECT_NAME}
)

# Export ament package
ament_package()
