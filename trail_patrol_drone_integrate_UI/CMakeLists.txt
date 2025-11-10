cmake_minimum_required(VERSION 3.8)
project(trail_patrol_drone_integrate_UI)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros_ign_interfaces REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(pluginlib REQUIRED)

# Qt5 automatic MOC (Meta-Object Compiler)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(QT_LIBRARIES Qt5::Widgets Qt5::Core)

# Removed drone_waypoint_navigator - not needed for this package

# ========== MISSION NODE ==========
add_executable(mission src/mission.cpp)

ament_target_dependencies(mission
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
)

# ========== NAVIGATION NODE ==========
add_executable(navigation src/navigation.cpp)

ament_target_dependencies(navigation
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

# ========== BRIDGE NODE (CONNECTS UI TO NAVIGATION) ==========
add_executable(drone_ui_navigation_bridge src/drone_ui_navigation_bridge.cpp)

ament_target_dependencies(drone_ui_navigation_bridge
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

# ========== Drone Control Panel Plugin ==========
add_library(drone_control_panel SHARED
  src/drone_control_panel.cpp
  src/drone_control_panel.hpp
)

ament_target_dependencies(drone_control_panel
  rclcpp
  std_msgs
  nav_msgs         
  geometry_msgs
  rviz_common
  rviz_rendering
  rviz_default_plugins
  pluginlib
)

target_link_libraries(drone_control_panel
  ${QT_LIBRARIES}
)

target_include_directories(drone_control_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

# Register the plugin
pluginlib_export_plugin_description_file(rviz_common src/plugin_description.xml)

# ========== Install Targets ==========

# Install executables
install(TARGETS
  mission
  navigation
  drone_ui_navigation_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Install plugin library
install(TARGETS drone_control_panel
  EXPORT drone_control_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install plugin description
install(FILES src/plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install other directories
install(
  DIRECTORY config launch models worlds urdf_drone
  DESTINATION share/${PROJECT_NAME}
)

# ========== Testing ==========
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ========== Export ==========
ament_export_dependencies(
  rviz_common
  pluginlib
)

ament_package()
