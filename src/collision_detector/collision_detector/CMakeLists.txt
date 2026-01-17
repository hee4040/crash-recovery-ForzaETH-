cmake_minimum_required(VERSION 3.5)
project(collision_detector)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(collision_detector_node src/collision_detector_node.cpp)

ament_target_dependencies(collision_detector_node
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
)

install(TARGETS
  collision_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

