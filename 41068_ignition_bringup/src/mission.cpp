#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <vector>
#include <cmath>
#include <chrono>

struct Waypoint {
    double x, y, z;
};

enum class ControlState {
    TURNING,
    MOVING
};

class Mission : public rclcpp::Node {
public:
    Mission() : Node("mission"), 
                current_waypoint_index_(0), 
                goal_reached_(false),
                control_state_(ControlState::TURNING) {
        // Publishers and subscribers
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,
            std::bind(&Mission::odom_callback, this, std::placeholders::_1)
        );
        
        // Control timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10Hz control loop
            std::bind(&Mission::control_loop, this)
        );
        
        // Initialize waypoints - modify these as needed
        // NOTE: Set z to match your drone spawn height (1.25) to maintain altitude

        waypoints_ = {
            {9.5, -12, 1},
            {7, -9.5, 1},
            {4, -6.5, 1},
            {2, -4.5, 1},
            {0, -1.5, 1},
            {3, 2.5, 1},
            {4, 4, 1},
            {6, 6, 1},
            {8, 8.5, 1},
            {11, 10.5, 1},
        };
        
        // Initialize state
        current_position_ = {0.0, 0.0, 0.0};
        current_yaw_ = 0.0;
        position_tolerance_ = 0.5; //  50cm tolerance for position
        turning_threshold_degrees_ = 30.0; // Stop to turn if angle > 30 degrees
        heading_tolerance_degrees_ = 5.0; // Face waypoint within 5 degrees
        
        RCLCPP_INFO(this->get_logger(), "Mission initialized with %zu waypoints", waypoints_.size());
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Extract position
        current_position_.x = msg->pose.pose.position.x;
        current_position_.y = msg->pose.pose.position.y;
        current_position_.z = msg->pose.pose.position.z;
        
        // Extract yaw from quaternion
        auto& q = msg->pose.pose.orientation;
        current_yaw_ = std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        );
    }
    
    double normalize_angle(double angle) {
        // Normalize angle to [-pi, pi]
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
    
    void control_loop() {
        // Check if all waypoints completed
        if (current_waypoint_index_ >= waypoints_.size()) {
            if (!goal_reached_) {
                RCLCPP_INFO(this->get_logger(), "All waypoints reached! Mission complete.");
                goal_reached_ = true;
            }
            // Send stop command
            auto cmd = geometry_msgs::msg::Twist();
            cmd_pub_->publish(cmd);
            return;
        }
        
        // Get current target waypoint
        const Waypoint& target = waypoints_[current_waypoint_index_];
        
        // Calculate distance to target
        double dx = target.x - current_position_.x;
        double dy = target.y - current_position_.y;
        double dz = target.z - current_position_.z;
        
        // Use 2D distance for navigation logic (ignore vertical)
        double horizontal_distance = std::sqrt(dx*dx + dy*dy);
        
        // Check if waypoint reached (using horizontal distance)
        if (horizontal_distance <= position_tolerance_) {
            RCLCPP_INFO(this->get_logger(), 
                "Waypoint %zu reached! Moving to next.", 
                current_waypoint_index_ + 1);
            current_waypoint_index_++;
            control_state_ = ControlState::TURNING; // Reset to turning for next waypoint
            
            // Send stop command briefly
            auto stop_cmd = geometry_msgs::msg::Twist();
            cmd_pub_->publish(stop_cmd);
            return;
        }
        
        // Calculate desired heading to target (only if not too close)
        double yaw_error = 0.0;
        double yaw_error_degrees = 0.0;
        
        // Only calculate heading if we're not extremely close (prevents oscillation)
        if (horizontal_distance > 1.0) {  // More than 1 meter away horizontally
            double target_yaw = std::atan2(dy, dx);
            yaw_error = normalize_angle(target_yaw - current_yaw_);
            yaw_error_degrees = std::abs(yaw_error) * 180.0 / M_PI;
        } else {
            // When very close, just keep current heading to avoid oscillation
            yaw_error_degrees = 0.0;
        }
        
        // State machine logic
        auto cmd = geometry_msgs::msg::Twist();
        
        switch (control_state_) {
            case ControlState::TURNING:
                {
                    // Pure turning state - no linear movement
                    if (yaw_error_degrees <= heading_tolerance_degrees_) {
                        // Heading is good, switch to moving
                        control_state_ = ControlState::MOVING;
                        RCLCPP_INFO(this->get_logger(), 
                            "Heading aligned (%.1f°), switching to MOVING state", 
                            yaw_error_degrees);
                    } else {
                        // Continue turning
                        double turn_speed = 0.5; // rad/s
                        
                        // Proportional control with minimum speed
                        double turn_gain = std::min(1.0, std::abs(yaw_error) / (M_PI / 4));
                        turn_gain = std::max(0.3, turn_gain); // Minimum 30% speed
                        
                        cmd.angular.z = turn_gain * turn_speed * (yaw_error > 0 ? 1.0 : -1.0);
                        
                        // No linear movement during turning
                        cmd.linear.x = 0.0;
                        cmd.linear.y = 0.0;
                        cmd.linear.z = 0.0;
                        
                        // Log turning progress periodically
                        static int turn_log_counter = 0;
                        if (++turn_log_counter % 10 == 0) { // Log every second
                            RCLCPP_INFO(this->get_logger(), 
                                "TURNING: Current position: (%.2f, %.2f, %.2f), Yaw error: %.1f°", 
                                current_position_.x, current_position_.y, current_position_.z,
                                yaw_error_degrees);
                        }
                    }
                }
                break;
                
            case ControlState::MOVING:
                {
                    // Check if we need to switch back to turning
                    // Only check heading if we're not in the close approach zone
                    if (horizontal_distance > 1.0 && yaw_error_degrees > turning_threshold_degrees_) {
                        control_state_ = ControlState::TURNING;
                        RCLCPP_INFO(this->get_logger(), 
                            "Heading deviation too large (%.1f°), switching to TURNING state", 
                            yaw_error_degrees);
                        
                        // Stop movement
                        cmd.linear.x = 0.0;
                        cmd.linear.y = 0.0;
                        cmd.linear.z = 0.0;
                    } else {
                        // Since we've aligned heading to face the target,
                        // we can simply move forward (much simpler and more robust)
                        double base_speed = 0.5; // Base speed
                        double min_speed = 0.08; // Minimum speed
                        
                        // Slow down when approaching waypoint
                        double speed_factor = 1.0;
                        if (horizontal_distance < 5.0) {
                            speed_factor = std::max(min_speed / base_speed, 
                                                   std::pow(horizontal_distance / 5.0, 0.7));
                        }
                        
                        // Further reduce speed when very close
                        if (horizontal_distance < 1.5) {
                            speed_factor *= 0.5; // Half speed in final approach
                        }
                        
                        double forward_speed = base_speed * speed_factor;
                        
                        // Set velocities in body frame
                        // Since we're facing the target, just move forward
                        cmd.linear.x = forward_speed;  // Forward velocity
                        cmd.linear.y = 0.0;            // No lateral movement
                        cmd.linear.z = 0.0;            // Maintain current altitude
                        
                        // Allow minor heading corrections while moving (but not when very close)
                        if (horizontal_distance > 1.0 && yaw_error_degrees > heading_tolerance_degrees_) {
                            // Gentle correction proportional to error
                            double correction_gain = 0.15; // Low gain for gentle correction
                            cmd.angular.z = yaw_error * correction_gain;
                            
                            // Limit maximum correction speed
                            double max_correction = 0.2; // rad/s
                            cmd.angular.z = std::max(-max_correction, 
                                                    std::min(max_correction, cmd.angular.z));
                        } else {
                            cmd.angular.z = 0.0;
                        }
                        
                        // Log progress periodically
                        static int move_log_counter = 0;
                        if (++move_log_counter % 20 == 0) { // Log every 2 seconds
                            RCLCPP_INFO(this->get_logger(), 
                                "MOVING: Current position: (%.2f, %.2f, %.2f), "
                                "Target waypoint %zu: (%.1f, %.1f, %.1f), Distance: %.2f m, Heading error: %.1f°", 
                                current_position_.x, current_position_.y, current_position_.z,
                                current_waypoint_index_ + 1, 
                                target.x, target.y, target.z, 
                                horizontal_distance, yaw_error_degrees);
                        }
                    }
                }
                break;
        }
        
        // Publish command
        cmd_pub_->publish(cmd);
    }
    
    // Member variables
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::vector<Waypoint> waypoints_;
    size_t current_waypoint_index_;
    bool goal_reached_;
    ControlState control_state_;
    
    Waypoint current_position_;
    double current_yaw_;
    
    // Tolerances and thresholds
    double position_tolerance_;
    double turning_threshold_degrees_;
    double heading_tolerance_degrees_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Mission>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}