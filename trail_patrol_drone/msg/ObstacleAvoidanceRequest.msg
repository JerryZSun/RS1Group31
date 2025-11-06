# Custom message for obstacle avoidance request from Navigation to Mission
# Save this in: trail_patrol_drone/msg/ObstacleAvoidanceRequest.msg

geometry_msgs/Point current_position  # Drone's current position
float64 best_clear_angle             # Best direction in radians (-999.0 if NO_PATH)
float64 obstacle_distance            # Distance to nearest obstacle in meters