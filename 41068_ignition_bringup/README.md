# 41068 Ignition Bringup

Bringup for *41068 Robotics Studio I*. Launches a Husky robot in a custom simulation world with trees and grass. We use **ROS2 Humble** and **Ignition Gazebo Fortress**.

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