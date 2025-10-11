# 41068 Ignition Bringup

Bringup for *41068 Robotics Studio I*. Launches a Husky robot in a custom simulation world with trees and grass. We use **ROS2 Humble** and **Ignition Gazebo Fortress**.

Added 05/09/2025: Also included a drone robot ("parrot"). Scroll down for instructions specifically for the drone.

Worlds are build from [Gazebo Fuel](https://app.gazebosim.org/fuel/models).

## Installation

First install some dependencies:

* If you haven't already, install ROS2 Humble. Follow the instructions here: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
* Install Gazebo
  ```bash
  sudo apt-get update && sudo apt-get install wget
  sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
  wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
  sudo apt-get update && sudo apt-get install ignition-fortress
  ```
* Install development tools and robot localisation
  ```bash
  sudo apt install ros-dev-tools ros-humble-robot-localization
  sudo apt install ros-humble-ros-ign ros-humble-ros-ign-interfaces
  sudo apt install ros-humble-turtlebot4-simulator ros-humble-irobot-create-nodes
  ```
* Make sure that your installation is up to date. This is particularly important if you installed ROS a long time ago, such as in another subject. If you get errors here, make sure to resolve these before continuing.
  ```bash
  sudo apt upgrade
  sudo apt update
  ```  

Now install this package:
* Create a new colcon workspace
  ```bash
  mkdir -p 41068_ws/src
  ```
* Copy this package to the `src` directory in this workspace
* Build package. If you get an error suggesting a missing dependency, make sure you have followed all of the above installation instructions correctly.
  ```bash
  source /opt/ros/humble/setup.bash
  cd 41068_ws
  colcon build --symlink-install
  ```
* Source workspace (if you add this to your ~/.bashrc, then you don't need to do this each time)
  ```bash
  source ~/41068_ws/install/setup.bash
  ```
* Launch basic trees world. It might take a little while to load the first time you run it since it is downloading world model resources. If it crashes the first time, try running it again.
  ```bash
  ros2 launch 41068_ignition_bringup 41068_ignition.launch.py
  ```
* As above with SLAM and autonomous navigation
  ```bash
  ros2 launch 41068_ignition_bringup 41068_ignition.launch.py slam:=true nav2:=true rviz:=true
  ```
* Change world with `world` argument. Must be the name of a `.sdf` file in `worlds`, but without file extension. Note this might also take a while the first time you run it since it is downloading extra model resources.
  ```bash
  ros2 launch 41068_ignition_bringup 41068_ignition.launch.py world:=large_demo
  ```
* And similarly, the larger world, and with SLAM and navigation:
  ```bash
  ros2 launch 41068_ignition_bringup 41068_ignition.launch.py slam:=true nav2:=true rviz:=true world:=large_demo
  ```
* When launching with rviz, you can send a waypoint to the robot by clicking the "2D Goal pose" and then a location in the map. The robot is navigating using the nav2 package. If it gets stuck, you can try the buttons in the Navigation 2 panel in the top right of RVIZ.

* You can also drive the robot using keyboard teleoperation by running the following in a separate terminal, then use the keys listed in the instructions to move the robot:
  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```

## Drone (added 05/09/2025)

By popular request, I've added a simple drone to the package, which requires a few modifications to get running:

* In the world file (large_demo.sdf or simple_trees.sdf, etc.), set the gravity to 0, so the drone doesn't fall out of the sky. I'll let you work out how to do that.

* I've created a separate launch file for the drone, which is almost the same, except spawns a "parrot" drone instead of the husky:
  ```bash
  ros2 launch 41068_ignition_bringup 41068_ignition_drone.launch.py slam:=true nav2:=true rviz:=true world:=large_demo
  ```

* At the moment, the drone will just fly at a fixed altitude. You can change the altitude in 41068_ignition_drone.launch.py. Find the `robot_spawner` node, and change change the `z` parameter, which is height above the ground where it is spawned.

* The drone's camera is setup to tilt 45 degrees downwards (looking slightly down towards the ground). You can change this in `urdf_drone/parrot.urdf.xacro`, find the `camera_joint` and change the "pitch" (you'll need to workout how to do this exactly).

* Since the drone has a very hard time navigating through the leaves of the trees, I've disabled the collisions of the drones. So at the moment, it is allowed to fly through objects. You can enable collisions by uncommenting the `collision` field in `urdf_drone/parrot.urdf.xacro`. Note that the default navigation stack doesn't work well in this situation, so you will need to further develop the collision avoidance planners for this new challenge!

* At the moment, you can only run the drone or the husky, not both. If you wish to spawn both at the same time, then you'll need to figure out how :)




## Errors

If you are getting errors, first check you are following the instructions correctly. Otherwise, read the error messages carefully and google it or discuss with your team or the teaching staff. Here's two errors I came across and fixes.

### Jump back in time

If you continuously get an error like:

```bash
Detected jump back in time. Clearing TF buffer
```

and you probably see things flashing in rviz, then this is probably due to the simulation clock time being reset constantly. This is likely caused by multiple gazebo instances running, perhaps a crashed gazebo in the background that didn't close properly. 

To fix this, I suggest restarting the computer. 

### Ogre Exception

If you get an error like:

```bash
[Ogre2RenderEngine.cc:989]  Unable to create the rendering window: OGRE EXCEPTION(3:RenderingAPIException): currentGLContext was specified with no current GL context in GLXWindow::create at /build/ogre-next-UFfg83/ogre-next-2.2.5+dfsg3/RenderSystems/GL3Plus/src/windowing/GLX/OgreGLXWindow.cpp (line 163)
```

I found [this thread](https://robotics.stackexchange.com/questions/111547/gazebo-crashes-immediately-segmentation-fault-address-not-mapped-to-object-0) which suggests to set a bash variable before launching Gazebo:

```bash
export QT_QPA_PLATFORM=xcb
```

Starting Cmake:

cmake_minimum_required(VERSION 3.8)
project(41068_ignition_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(ros_ign_interfaces REQUIRED)

install(
  DIRECTORY config launch models urdf worlds urdf_drone
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
a
ament_package()

Usefull Commands:

ros2 topic list -t
ros2 node list
ros2 node info /robot_localization


Result:

jsunne@DESKTOP-AO6197G:~/git/RS1Group31/41068_ignition_bringup$ ros2 topic list -t
ros2 node list
ros2 node info /robot_localization
/behavior_server/transition_event [lifecycle_msgs/msg/TransitionEvent]
/behavior_tree_log [nav2_msgs/msg/BehaviorTreeLog]
/bond [bond/msg/Status]
/bt_navigator/transition_event [lifecycle_msgs/msg/TransitionEvent]
/camera/depth/image [sensor_msgs/msg/Image]
/camera/depth/points [sensor_msgs/msg/PointCloud2]
/camera/image [sensor_msgs/msg/Image]
/clicked_point [geometry_msgs/msg/PointStamped]
/clock [rosgraph_msgs/msg/Clock]
/cmd_vel [geometry_msgs/msg/Twist]
/cmd_vel_nav [geometry_msgs/msg/Twist]
/cmd_vel_teleop [geometry_msgs/msg/Twist]
/controller_server/transition_event [lifecycle_msgs/msg/TransitionEvent]
/cost_cloud [sensor_msgs/msg/PointCloud2]
/diagnostics [diagnostic_msgs/msg/DiagnosticArray]
/evaluation [dwb_msgs/msg/LocalPlanEvaluation]
/global_costmap/costmap [nav_msgs/msg/OccupancyGrid]
/global_costmap/costmap_raw [nav2_msgs/msg/Costmap]
/global_costmap/costmap_updates [map_msgs/msg/OccupancyGridUpdate]
/global_costmap/footprint [geometry_msgs/msg/Polygon]
/global_costmap/global_costmap/transition_event [lifecycle_msgs/msg/TransitionEvent]
/global_costmap/published_footprint [geometry_msgs/msg/PolygonStamped]
/goal_pose [geometry_msgs/msg/PoseStamped]
/imu [sensor_msgs/msg/Imu]
/initialpose [geometry_msgs/msg/PoseWithCovarianceStamped]
/joint_states [sensor_msgs/msg/JointState]
/local_costmap/costmap [nav_msgs/msg/OccupancyGrid]
/local_costmap/costmap_raw [nav2_msgs/msg/Costmap]
/local_costmap/costmap_updates [map_msgs/msg/OccupancyGridUpdate]
/local_costmap/footprint [geometry_msgs/msg/Polygon]
/local_costmap/local_costmap/transition_event [lifecycle_msgs/msg/TransitionEvent]
/local_costmap/published_footprint [geometry_msgs/msg/PolygonStamped]
/local_plan [nav_msgs/msg/Path]
/map [nav_msgs/msg/OccupancyGrid]
/map_metadata [nav_msgs/msg/MapMetaData]
/map_updates [map_msgs/msg/OccupancyGridUpdate]
/marker [visualization_msgs/msg/MarkerArray]
/odom [nav_msgs/msg/Odometry]
/odometry [nav_msgs/msg/Odometry]
/odometry/filtered [nav_msgs/msg/Odometry]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/plan [nav_msgs/msg/Path]
/plan_smoothed [nav_msgs/msg/Path]
/planner_server/transition_event [lifecycle_msgs/msg/TransitionEvent]
/pose [geometry_msgs/msg/PoseWithCovarianceStamped]
/preempt_teleop [std_msgs/msg/Empty]
/received_global_plan [nav_msgs/msg/Path]
/robot_description [std_msgs/msg/String]
/rosout [rcl_interfaces/msg/Log]
/scan [sensor_msgs/msg/LaserScan]
/scan/points [sensor_msgs/msg/PointCloud2]
/set_pose [geometry_msgs/msg/PoseWithCovarianceStamped]
/slam_toolbox/feedback [visualization_msgs/msg/InteractiveMarkerFeedback]
/slam_toolbox/graph_visualization [visualization_msgs/msg/MarkerArray]
/slam_toolbox/scan_visualization [sensor_msgs/msg/LaserScan]
/slam_toolbox/update [visualization_msgs/msg/InteractiveMarkerUpdate]
/smoother_server/transition_event [lifecycle_msgs/msg/TransitionEvent]
/speed_limit [nav2_msgs/msg/SpeedLimit]
/tf [tf2_msgs/msg/TFMessage]
/tf_static [tf2_msgs/msg/TFMessage]
/transformed_global_plan [nav_msgs/msg/Path]
/velocity_smoother/transition_event [lifecycle_msgs/msg/TransitionEvent]
/waypoint_follower/transition_event [lifecycle_msgs/msg/TransitionEvent]
/waypoints [visualization_msgs/msg/MarkerArray]
/behavior_server
/bt_navigator
/bt_navigator_navigate_through_poses_rclcpp_node
/bt_navigator_navigate_to_pose_rclcpp_node
/controller_server
/global_costmap/global_costmap
/lifecycle_manager_navigation
/local_costmap/local_costmap
/planner_server
/robot_localization
/robot_state_publisher
/ros_gz_bridge
/rviz
/rviz_navigation_dialog_action_client
/slam_toolbox
/smoother_server
/transform_listener_impl_559709feab70
/transform_listener_impl_560d5e3ddd70
/transform_listener_impl_56436da55010
/transform_listener_impl_5aa7244b3150
/transform_listener_impl_5f401027f2b0
/transform_listener_impl_6094ca4111a0
/transform_listener_impl_62139a9a4ab0
/velocity_smoother
/waypoint_follower
/robot_localization
  Subscribers:
    /clock: rosgraph_msgs/msg/Clock
    /imu: sensor_msgs/msg/Imu
    /odometry: nav_msgs/msg/Odometry
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /set_pose: geometry_msgs/msg/PoseWithCovarianceStamped
  Publishers:
    /diagnostics: diagnostic_msgs/msg/DiagnosticArray
    /odometry/filtered: nav_msgs/msg/Odometry
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /tf: tf2_msgs/msg/TFMessage
  Service Servers:
    /enable: std_srvs/srv/Empty
    /robot_localization/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /robot_localization/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /robot_localization/get_parameters: rcl_interfaces/srv/GetParameters
    /robot_localization/list_parameters: rcl_interfaces/srv/ListParameters
    /robot_localization/set_parameters: rcl_interfaces/srv/SetParameters
    /robot_localization/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    /set_pose: robot_localization/srv/SetPose
    /toggle: robot_localization/srv/ToggleFilterProcessing
  Service Clients:

  Action Servers:

  Action Clients: