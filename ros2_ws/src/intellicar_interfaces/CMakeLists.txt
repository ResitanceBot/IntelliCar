cmake_minimum_required(VERSION 3.5)
project(intellicar_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BboxTrafficLight.msg"
)

ament_export_dependencies(rosidl_default_runtime)

install(DIRECTORY
  msg
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
