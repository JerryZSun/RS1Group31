cmake_minimum_required(VERSION 3.8)
project(trail_patrol_drone)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros_ign_interfaces REQUIRED)

# Create mission executable
add_executable(mission src/mission.cpp)
ament_target_dependencies(mission
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
)

# Create navigation executable
add_executable(navigation src/navigation.cpp)
ament_target_dependencies(navigation
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
)

# Install executables
install(TARGETS
  mission
  navigation
  DESTINATION lib/${PROJECT_NAME}
)

# Install other directories
install(
  DIRECTORY config launch models urdf worlds urdf_drone
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()