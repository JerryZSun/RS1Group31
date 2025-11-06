#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <chrono>

enum class ControlState {
    IDLE,
    CLIMBING,
    TURNING,
    MOVING,
    OBSTACLE_DETECTED
};

class Navigation : public rclcpp::Node {
public:
    Navigation() : Node("navigation"),
                   control_state_(ControlState::IDLE),
                   has_target_(false),
                   obstacle_detected_(false) {
        
        // Publishers
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        waypoint_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/navigation/waypoint_reached", 10);
        obstacle_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/navigation/obstacle_detected", 10);
        
        // Subscribers
        waypoint_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mission/current_waypoint", 10,
            std::bind(&Navigation::waypoint_callback, this, std::placeholders::_1)
        );
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,
            std::bind(&Navigation::odom_callback, this, std::placeholders::_1)
        );
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&Navigation::scan_callback, this, std::placeholders::_1)
        );
        
        // Control timer - 10Hz control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Navigation::control_loop, this)
        );
        
        // Initialize parameters
        current_position_ = {0.0, 0.0, 0.0};
        target_position_ = {0.0, 0.0, 0.0};
        current_yaw_ = 0.0;
        
        position_tolerance_ = 0.5; // 50cm tolerance
        altitude_tolerance_ = 0.3; // 30cm altitude tolerance
        climbing_threshold_ = 2.0; // Enter CLIMBING state if altitude diff > 2m
        turning_threshold_degrees_ = 30.0;
        heading_tolerance_degrees_ = 5.0;
        obstacle_threshold_ = 2.0; // Detection distance - 2m gives ~1.5m buffer at 0.5m/s speed
        
        RCLCPP_INFO(this->get_logger(), "Navigation node initialized");
        RCLCPP_INFO(this->get_logger(), "Obstacle detection: distance=%.2fm, cone=±20°, ONLY when MOVING and aligned", 
                    obstacle_threshold_);
    }

private:
    void waypoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        target_position_.x = msg->pose.position.x;
        target_position_.y = msg->pose.position.y;
        target_position_.z = msg->pose.position.z;
        
        has_target_ = true;
        obstacle_detected_ = false;
        
        // Determine initial state based on altitude difference
        double altitude_error = std::abs(target_position_.z - current_position_.z);
        if (altitude_error > climbing_threshold_) {
            control_state_ = ControlState::CLIMBING;
            RCLCPP_INFO(this->get_logger(), 
                "New waypoint received: (%.2f, %.2f, %.2f) - Large altitude change, entering CLIMBING state", 
                target_position_.x, target_position_.y, target_position_.z);
        } else {
            control_state_ = ControlState::TURNING;
            RCLCPP_INFO(this->get_logger(), 
                "New waypoint received: (%.2f, %.2f, %.2f)", 
                target_position_.x, target_position_.y, target_position_.z);
        }
    }
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
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
    
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // CRITICAL: Only check for obstacles when MOVING and well-aligned toward target
        if (control_state_ != ControlState::MOVING) {
            return;
        }
        
        // Calculate if we're aligned with target
        double dx = target_position_.x - current_position_.x;
        double dy = target_position_.y - current_position_.y;
        double target_yaw = std::atan2(dy, dx);
        double yaw_error = normalize_angle(target_yaw - current_yaw_);
        double yaw_error_degrees = std::abs(yaw_error) * 180.0 / M_PI;
        
        // Only check for obstacles if heading is well-aligned (< 15 degrees off)
        if (yaw_error_degrees > 15.0) {
            return;
        }
        
        size_t ranges_size = msg->ranges.size();
        
        // Narrow cone: 40 degrees (20 each side) - focused on direct path
        int front_range = ranges_size / 9;  // 40 degrees total
        
        bool obstacle_in_path = false;
        float closest_range = std::numeric_limits<float>::max();
        int detection_count = 0;
        
        for (int i = -front_range; i <= front_range; ++i) {
            int idx = (ranges_size / 2 + i + ranges_size) % ranges_size;
            float range = msg->ranges[idx];
            
            if (std::isnan(range) || std::isinf(range) || range < 0.2) {
                continue;
            }
            
            // Check if within detection threshold
            if (range < obstacle_threshold_ && range > 0.2) {
                detection_count++;
                closest_range = std::min(closest_range, range);
            }
        }
        
        // Require at least 5 detections in the cone
        if (detection_count >= 5) {
            obstacle_in_path = true;
        }
        
        // Update obstacle detection state
        if (obstacle_in_path && !obstacle_detected_) {
            obstacle_detected_ = true;
            
            auto obs_msg = std_msgs::msg::Bool();
            obs_msg.data = true;
            obstacle_pub_->publish(obs_msg);
            
            RCLCPP_WARN(this->get_logger(), 
                "OBSTACLE DETECTED at %.2fm from position (%.2f, %.2f, %.2f)! %d detections, heading error: %.1f°. Stopping.", 
                closest_range, current_position_.x, current_position_.y, current_position_.z, 
                detection_count, yaw_error_degrees);
            
            control_state_ = ControlState::OBSTACLE_DETECTED;
        } else if (!obstacle_in_path && obstacle_detected_) {
            obstacle_detected_ = false;
            
            auto obs_msg = std_msgs::msg::Bool();
            obs_msg.data = false;
            obstacle_pub_->publish(obs_msg);
            
            RCLCPP_INFO(this->get_logger(), "Obstacle cleared, resuming navigation");
            
            if (has_target_) {
                // Re-evaluate state based on altitude
                double altitude_error = std::abs(target_position_.z - current_position_.z);
                if (altitude_error > climbing_threshold_) {
                    control_state_ = ControlState::CLIMBING;
                } else {
                    control_state_ = ControlState::TURNING;
                }
            }
        }
    }
    
    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
    
    void control_loop() {
        auto cmd = geometry_msgs::msg::Twist();
        
        if (!has_target_) {
            // No target, remain idle
            cmd_pub_->publish(cmd);
            return;
        }
        
        // Calculate distance to target
        double dx = target_position_.x - current_position_.x;
        double dy = target_position_.y - current_position_.y;
        double dz = target_position_.z - current_position_.z;
        double horizontal_distance = std::sqrt(dx*dx + dy*dy);
        double altitude_error = dz;
        
        // Check if waypoint reached (both horizontal and altitude)
        if (horizontal_distance <= position_tolerance_ && 
            std::abs(altitude_error) <= altitude_tolerance_) {
            RCLCPP_INFO(this->get_logger(), 
                "Waypoint reached at (%.2f, %.2f, %.2f)", 
                current_position_.x, current_position_.y, current_position_.z);
            
            auto reached_msg = std_msgs::msg::Bool();
            reached_msg.data = true;
            waypoint_reached_pub_->publish(reached_msg);
            
            has_target_ = false;
            control_state_ = ControlState::IDLE;
            cmd_pub_->publish(cmd); // Stop
            return;
        }
        
        // Handle obstacle state
        if (control_state_ == ControlState::OBSTACLE_DETECTED) {
            // Stop and wait, but maintain altitude
            cmd.linear.z = calculate_altitude_control(altitude_error);
            cmd_pub_->publish(cmd);
            return;
        }
        
        // Calculate heading to target
        double yaw_error = 0.0;
        double yaw_error_degrees = 0.0;
        
        if (horizontal_distance > 1.0) {
            double target_yaw = std::atan2(dy, dx);
            yaw_error = normalize_angle(target_yaw - current_yaw_);
            yaw_error_degrees = std::abs(yaw_error) * 180.0 / M_PI;
        }
        
        // Always apply gentle altitude correction (to fight gravity when enabled)
        cmd.linear.z = calculate_altitude_control(altitude_error);
        
        // Check if we need to prioritize climbing
        if (std::abs(altitude_error) > climbing_threshold_ && 
            control_state_ != ControlState::CLIMBING) {
            control_state_ = ControlState::CLIMBING;
            RCLCPP_INFO(this->get_logger(), 
                "Large altitude error (%.2fm), entering CLIMBING state", altitude_error);
        }
        
        // State machine for control
        switch (control_state_) {
            case ControlState::CLIMBING:
                {
                    // Focus on altitude correction first
                    if (std::abs(altitude_error) <= altitude_tolerance_) {
                        // Altitude reached, transition to turning
                        control_state_ = ControlState::TURNING;
                        RCLCPP_INFO(this->get_logger(), 
                            "Target altitude reached, transitioning to TURNING");
                    } else {
                        // Pure vertical movement (cmd.linear.z already set above)
                        cmd.linear.x = 0.0;
                        cmd.linear.y = 0.0;
                        cmd.angular.z = 0.0;
                        
                        static int climb_log_counter = 0;
                        if (++climb_log_counter % 10 == 0) {
                            RCLCPP_INFO(this->get_logger(), 
                                "CLIMBING: Current altitude: %.2fm, Target: %.2fm, Error: %.2fm", 
                                current_position_.z, target_position_.z, altitude_error);
                        }
                    }
                }
                break;
                
            case ControlState::TURNING:
                {
                    if (yaw_error_degrees <= heading_tolerance_degrees_) {
                        control_state_ = ControlState::MOVING;
                        RCLCPP_INFO(this->get_logger(), 
                            "Heading aligned (%.1f°), moving forward", 
                            yaw_error_degrees);
                    } else {
                        // Pure turning (altitude control still active via cmd.linear.z)
                        double turn_speed = 0.5; // rad/s
                        double turn_gain = std::min(1.0, std::abs(yaw_error) / (M_PI / 4));
                        turn_gain = std::max(0.3, turn_gain);
                        
                        cmd.angular.z = turn_gain * turn_speed * (yaw_error > 0 ? 1.0 : -1.0);
                        cmd.linear.x = 0.0;
                        cmd.linear.y = 0.0;
                        
                        static int turn_log_counter = 0;
                        if (++turn_log_counter % 10 == 0) {
                            RCLCPP_INFO(this->get_logger(), 
                                "TURNING: Yaw error: %.1f°, Altitude: %.2fm", 
                                yaw_error_degrees, current_position_.z);
                        }
                    }
                }
                break;
                
            case ControlState::MOVING:
                {
                    // Check if altitude error became large again
                    if (std::abs(altitude_error) > climbing_threshold_) {
                        control_state_ = ControlState::CLIMBING;
                        RCLCPP_INFO(this->get_logger(), 
                            "Altitude error increased, re-entering CLIMBING state");
                        cmd.linear.x = 0.0;
                        break;
                    }
                    
                    // Check if need to turn again
                    if (horizontal_distance > 1.0 && yaw_error_degrees > turning_threshold_degrees_) {
                        control_state_ = ControlState::TURNING;
                        RCLCPP_INFO(this->get_logger(), 
                            "Heading deviation too large (%.1f°), re-aligning", 
                            yaw_error_degrees);
                        cmd.linear.x = 0.0;
                    } else {
                        // Move forward with speed modulation
                        double base_speed = 0.5;
                        double min_speed = 0.08;
                        double speed_factor = 1.0;
                        
                        if (horizontal_distance < 5.0) {
                            speed_factor = std::max(min_speed / base_speed, 
                                                   std::pow(horizontal_distance / 5.0, 0.7));
                        }
                        
                        if (horizontal_distance < 1.5) {
                            speed_factor *= 0.5;
                        }
                        
                        cmd.linear.x = base_speed * speed_factor;
                        
                        // Minor heading corrections
                        if (horizontal_distance > 1.0 && yaw_error_degrees > heading_tolerance_degrees_) {
                            double correction_gain = 0.15;
                            cmd.angular.z = yaw_error * correction_gain;
                            cmd.angular.z = std::max(-0.2, std::min(0.2, cmd.angular.z));
                        } else {
                            cmd.angular.z = 0.0;
                        }
                        
                        static int move_log_counter = 0;
                        if (++move_log_counter % 20 == 0) {
                            RCLCPP_INFO(this->get_logger(), 
                                "MOVING: Distance: %.2fm, Altitude: %.2fm (error: %.2fm), Heading: %.1f°", 
                                horizontal_distance, current_position_.z, altitude_error, yaw_error_degrees);
                        }
                    }
                }
                break;
                
            case ControlState::IDLE:
            default:
                // Do nothing
                break;
        }
        
        cmd_pub_->publish(cmd);
    }
    
    double calculate_altitude_control(double altitude_error) {
        // Gentle proportional control for altitude
        // This fights gravity and provides smooth altitude adjustments
        double kp_altitude = 0.3; // Proportional gain
        double max_climb_speed = 0.5; // Max vertical speed (m/s)
        
        double climb_speed = kp_altitude * altitude_error;
        
        // Clamp to max speed
        climb_speed = std::max(-max_climb_speed, std::min(max_climb_speed, climb_speed));
        
        // Add small constant to fight gravity (when gravity is enabled)
        // You can tune this value when you turn gravity on
        double gravity_compensation = 0.0; // Set to ~0.1-0.2 when gravity enabled
        
        return climb_speed + gravity_compensation;
    }
    
    // Member variables
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr waypoint_reached_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_pub_;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    
    struct Position {
        double x, y, z;
    };
    
    Position current_position_;
    Position target_position_;
    double current_yaw_;
    
    ControlState control_state_;
    bool has_target_;
    bool obstacle_detected_;
    
    double position_tolerance_;
    double altitude_tolerance_;
    double climbing_threshold_;
    double turning_threshold_degrees_;
    double heading_tolerance_degrees_;
    double obstacle_threshold_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Navigation>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}