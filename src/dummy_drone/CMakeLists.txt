cmake_minimum_required(VERSION 3.5)
project(dummy_drone)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(dummy_drone_controller src/dummy_drone_controller.cpp)
ament_target_dependencies(dummy_drone_controller rclcpp geometry_msgs)

install(TARGETS
  dummy_drone_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
