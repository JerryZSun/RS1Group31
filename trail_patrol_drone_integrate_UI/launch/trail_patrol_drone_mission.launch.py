from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('trail_patrol_drone_integrate_UI')
    rviz_config = os.path.join(pkg_share, 'config', '41068.rviz')

    # Launch arguments
    slam_arg = DeclareLaunchArgument('slam', default_value='true')
    nav2_arg = DeclareLaunchArgument('nav2', default_value='true')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    world_arg = DeclareLaunchArgument('world', default_value='large_demo')

    # Include base simulation (Gazebo + Drone)
    include_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', '41068_ignition_drone.launch.py')
        ),
        launch_arguments={
            'slam': LaunchConfiguration('slam'),
            'nav2': LaunchConfiguration('nav2'),
            'rviz': LaunchConfiguration('rviz'),
            'world': LaunchConfiguration('world'),
        }.items()
    )

    # ========================================
    # JERRY'S MISSION PLANNER
    # ========================================
    # High-level mission with hardcoded waypoints
    # Publishes to: /mission/current_waypoint (to navigation)
    jerry_mission = Node(
        package='trail_patrol_drone_integrate_UI',
        executable='mission',
        name='mission',
        output='screen'
    )

    # ========================================
    # JERRY'S NAVIGATION CONTROLLER
    # ========================================
    # Low-level navigation that follows waypoints
    jerry_navigation = Node(
        package='trail_patrol_drone_integrate_UI',
        executable='navigation',
        name='navigation',
        output='screen'
    )

    # ========================================
    # UI NAVIGATION BRIDGE (feeds RViz panel)
    # ========================================
    ui_bridge = Node(
        package='trail_patrol_drone_integrate_UI',
        executable='drone_ui_navigation_bridge',
        name='drone_ui_navigation_bridge',
        output='screen'
    )

    return LaunchDescription([
        slam_arg,
        nav2_arg,
        rviz_arg,
        world_arg,
        include_bringup,
        jerry_mission,
        jerry_navigation,
        ui_bridge,
    ])
