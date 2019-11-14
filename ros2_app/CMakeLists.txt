cmake_minimum_required(VERSION 3.5)
project(ros2_px4_profiler)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(ros2_px4_profiler ros2_px4_profiler.cpp)
ament_target_dependencies(ros2_px4_profiler rclcpp px4_msgs)

install(TARGETS
  ros2_px4_profiler
  DESTINATION lib/${PROJECT_NAME})

ament_package()
