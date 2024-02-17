cmake_minimum_required(VERSION 3.5)
project(gamepad_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(gamepad_driver_node src/gamepad_driver_node.cpp)
ament_target_dependencies(gamepad_driver_node rclcpp geometry_msgs sensor_msgs)

install(TARGETS
  gamepad_driver_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
