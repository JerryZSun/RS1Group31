/**
 * @file navigation.cpp
 * @brief Low-level navigation controller for autonomous drone flight
 *
 * This file implements the Navigation class which handles:
 * - PID-style waypoint navigation with altitude, heading, and position control
 * - Multi-threshold obstacle detection (emergency, normal, avoidance)
 * - Intelligent path scanning and clearance detection
 * - Velocity command generation for drone control
 * - State machine for navigation phases (climbing, turning, moving)
 *
 * @author RS1 Group 31
 * @date 2025
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <chrono>
#include <algorithm>
#include <limits>

/**
 * @brief Navigation control state enumeration
 *
 * Defines the different states for the navigation controller's state machine,
 * controlling how the drone approaches waypoints and handles obstacles.
 */
enum class ControlState {
    IDLE,                   ///< No active waypoint, drone stationary
    CLIMBING,               ///< Adjusting altitude to target (vertical only)
    TURNING,                ///< Rotating to face target heading (rotation only)
    MOVING,                 ///< Moving forward toward waypoint
    OBSTACLE_SCANNING,      ///< Scanning environment for clear path (1 second)
    OBSTACLE_STOP,          ///< Stopped due to obstacle, waiting for avoidance waypoint
    AVOIDANCE_MANEUVER,     ///< Executing avoidance maneuver waypoint
    OBSTACLE_DETECTED,      ///< Obstacle detected flag state
    EMERGENCY_LANDING       ///< Emergency descent to ground
};

/**
 * @brief Main navigation controller class for drone flight control
 *
 * This class manages low-level drone navigation including:
 * - Waypoint following with state machine control
 * - Three-tier obstacle detection system:
 *   - Emergency: < 0.5m (immediate fly-over)
 *   - Normal: < 2.0m (scan and choose avoidance)
 *   - Avoidance: < 1.5m (during avoidance maneuvers)
 * - Left/right path scanning for fly-around decisions
 * - Velocity command generation (linear and angular)
 * - UI control integration (pause, resume, emergency stop)
 *
 * The navigation node receives waypoints from the mission node and publishes
 * cmd_vel commands to control the drone's movement.
 */
class Navigation : public rclcpp::Node {
public:
    Navigation() : Node("navigation"),
                   control_state_(ControlState::IDLE),
                   has_target_(false),
                   obstacle_detected_(false),
                   emergency_triggered_(false),
                   is_paused_(false),
                   in_avoidance_mode_(false),
                   scan_start_time_(this->now()),
                   last_status_print_(this->now()) {
        
        // Publishers
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        waypoint_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/navigation/waypoint_reached", 10);
        obstacle_detected_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/navigation/obstacle_detected", 10);
        avoidance_direction_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/navigation/avoidance_direction", 10);
        
        // Subscribers
        waypoint_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mission/current_waypoint", 10,
            std::bind(&Navigation::waypoint_callback, this, std::placeholders::_1)
        );
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&Navigation::odom_callback, this, std::placeholders::_1)
        );
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&Navigation::scan_callback, this, std::placeholders::_1)
        );

        avoidance_mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/mission/avoidance_mode", 10,
            std::bind(&Navigation::avoidance_mode_callback, this, std::placeholders::_1)
        );

        // UI control subscribers
        pause_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/drone/pause", 10,
            std::bind(&Navigation::pause_callback, this, std::placeholders::_1)
        );

        resume_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/drone/resume", 10,
            std::bind(&Navigation::resume_callback, this, std::placeholders::_1)
        );

        emergency_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/drone/emergency_stop", 10,
            std::bind(&Navigation::emergency_stop_callback, this, std::placeholders::_1)
        );

        // Control timer - 10Hz control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Navigation::control_loop, this)
        );

        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5000),
            std::bind(&Navigation::print_status, this)
        );
        
        // Initialize parameters
        current_position_ = {0.0, 0.0, 0.0};
        target_position_ = {0.0, 0.0, 0.0};
        current_yaw_ = 0.0;
        
        position_tolerance_ = 0.5; // 50cm tolerance
        altitude_tolerance_ = 1.0; // 1m altitude tolerance (increased for stability)
        climbing_threshold_ = 5.0; // Enter CLIMBING state if altitude diff > 5m (increased)
        turning_threshold_degrees_ = 30.0;
        heading_tolerance_degrees_ = 5.0;

        normal_obstacle_threshold_ = 2.0;
        avoidance_obstacle_threshold_ = 1.5;
        emergency_obstacle_threshold_ = 0.5;
        
        RCLCPP_INFO(this->get_logger(), "Navigation node initialized");
        RCLCPP_INFO(this->get_logger(), "Obstacle detection thresholds - normal %.1fm, avoidance %.1fm, emergency %.1fm",
                    normal_obstacle_threshold_, avoidance_obstacle_threshold_, emergency_obstacle_threshold_);
    }

private:
    void waypoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        target_position_.x = msg->pose.position.x;
        target_position_.y = msg->pose.position.y;
        target_position_.z = msg->pose.position.z;

        has_target_ = true;

        if (control_state_ != ControlState::AVOIDANCE_MANEUVER &&
            control_state_ != ControlState::OBSTACLE_STOP) {
            obstacle_detected_ = false;
            emergency_triggered_ = false;
        }

        if (in_avoidance_mode_) {
            control_state_ = ControlState::AVOIDANCE_MANEUVER;
            RCLCPP_INFO(this->get_logger(),
                "Avoidance waypoint received: (%.2f, %.2f, %.2f) - detection disabled",
                target_position_.x, target_position_.y, target_position_.z);
        } else {
            // Determine initial state based on altitude difference
            double altitude_error = std::abs(target_position_.z - current_position_.z);
            if (altitude_error > climbing_threshold_) {
                control_state_ = ControlState::CLIMBING;
                RCLCPP_INFO(this->get_logger(),
                    "New waypoint received: (%.2f, %.2f, %.2f) - entering CLIMBING state",
                    target_position_.x, target_position_.y, target_position_.z);
            } else {
                control_state_ = ControlState::TURNING;
                RCLCPP_INFO(this->get_logger(),
                    "New waypoint received: (%.2f, %.2f, %.2f)",
                    target_position_.x, target_position_.y, target_position_.z);
            }
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

    void avoidance_mode_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        in_avoidance_mode_ = msg->data;
        if (in_avoidance_mode_) {
            RCLCPP_INFO(this->get_logger(), "Avoidance mode ACTIVE - suppressing obstacle checks");
        } else {
            RCLCPP_INFO(this->get_logger(), "Avoidance mode INACTIVE - normal detection restored");
        }
    }
    
    /**
     * @brief Processes laser scan data for multi-level obstacle detection
     *
     * Implements a three-tier obstacle detection system:
     * 1. Emergency detection (< 0.5m): Always active, triggers immediate fly-over
     * 2. Normal detection (< 2.0m): Active when MOVING and aligned, triggers 1s scan
     * 3. Path scanning: After 1s scan, determines best avoidance direction
     *
     * Detection uses a ±20° cone in front of the drone (40° total field of view).
     * Requires 5+ laser points detecting obstacle for normal triggering,
     * 3+ points for emergency triggering.
     *
     * @param msg Laser scan message containing 360° range measurements
     *
     * @note Skips normal detection during avoidance maneuvers or after emergency trigger
     * @note Only checks obstacles when drone is aligned within 15° of target heading
     * @see check_emergency_obstacle(), scan_for_clear_path(), check_segment_clear()
     */
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = msg;

        if (!has_target_) return;

        // EMERGENCY CHECK - Always active at 0.5m
        if (check_emergency_obstacle(msg)) {
            if (control_state_ != ControlState::OBSTACLE_STOP || !emergency_triggered_) {
                emergency_triggered_ = true;
                obstacle_detected_ = true;
                control_state_ = ControlState::OBSTACLE_STOP;

                RCLCPP_ERROR(this->get_logger(), "🚨 EMERGENCY: Obstacle within 0.5m - FLY-OVER initiated!");

                // Trigger fly-over (direction = 0)
                auto direction_msg = std_msgs::msg::Int32();
                direction_msg.data = 0;
                avoidance_direction_pub_->publish(direction_msg);

                auto obstacle_msg = std_msgs::msg::Bool();
                obstacle_msg.data = true;
                obstacle_detected_pub_->publish(obstacle_msg);
            }
            return;
        }

        // Skip normal detection during avoidance or emergency
        if (control_state_ == ControlState::AVOIDANCE_MANEUVER || emergency_triggered_) {
            return;
        }

        // Handle scanning state (1 second scan)
        if (control_state_ == ControlState::OBSTACLE_SCANNING) {
            auto elapsed = (this->now() - scan_start_time_).seconds();
            if (elapsed >= 1.0) {
                scan_for_clear_path(msg);
            }
            return;
        }

        // Normal obstacle detection (only when MOVING and aligned)
        if (control_state_ != ControlState::MOVING) return;

        double dx = target_position_.x - current_position_.x;
        double dy = target_position_.y - current_position_.y;
        double target_yaw = std::atan2(dy, dx);
        double yaw_error = normalize_angle(target_yaw - current_yaw_);
        double yaw_error_degrees = std::abs(yaw_error) * 180.0 / M_PI;

        if (yaw_error_degrees > 15.0) return;

        // Check ±20° cone in front
        int cone_range = msg->ranges.size() / 9;
        size_t ranges_size = msg->ranges.size();
        float closest_range = std::numeric_limits<float>::max();
        int detection_count = 0;

        for (int i = -cone_range; i <= cone_range; ++i) {
            int idx = (i + ranges_size) % ranges_size;
            float range = msg->ranges[idx];

            if (std::isnan(range) || std::isinf(range) || range < 0.2) continue;

            if (range < normal_obstacle_threshold_ && range > 0.2) {
                detection_count++;
                closest_range = std::min(closest_range, range);
            }
        }

        // Trigger if 5+ points detect obstacle
        if (detection_count >= 5 && !obstacle_detected_) {
            obstacle_detected_ = true;
            control_state_ = ControlState::OBSTACLE_SCANNING;
            scan_start_time_ = this->now();

            RCLCPP_WARN(this->get_logger(), "OBSTACLE DETECTED at %.2fm - Starting scan", closest_range);

            auto obstacle_msg = std_msgs::msg::Bool();
            obstacle_msg.data = true;
            obstacle_detected_pub_->publish(obstacle_msg);
        }
    }
    /**
     * @brief Checks for obstacles within emergency threshold (0.5m)
     *
     * Scans a ±20° cone in front of the drone for very close obstacles.
     * This check is always active regardless of navigation state to ensure
     * safety. Triggers immediate fly-over protocol if detected.
     *
     * @param msg Laser scan message with range data
     * @return true if 3 or more laser points detect obstacle within 0.5m
     *
     * @note Always active - not suppressed during avoidance or other states
     * @note Lower threshold (3 points) than normal detection for safety
     * @see scan_callback()
     */
    bool check_emergency_obstacle(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        size_t ranges_size = msg->ranges.size();
        int front_range = ranges_size / 9; // ±20°
        int detection_count = 0;
        float closest_range = std::numeric_limits<float>::max();

        for (int i = -front_range; i <= front_range; ++i) {
            int idx = (i + ranges_size) % ranges_size;
            float range = msg->ranges[idx];

            if (std::isnan(range) || std::isinf(range) || range < 0.2) continue;

            if (range < emergency_obstacle_threshold_ && range > 0.2) {
                detection_count++;
                closest_range = std::min(closest_range, range);
            }
        }

        return detection_count >= 3;
    }

    /**
     * @brief Scans left and right sides to determine best avoidance direction
     *
     * After obstacle detection triggers a 1-second scan, this function analyzes
     * 30° segments on the left (90°) and right (270°) sides of the drone to
     * determine which direction has a clear path.
     *
     * Avoidance decision logic:
     * - Right clear → Direction 1 (fly-around right)
     * - Left clear → Direction 2 (fly-around left)
     * - Neither clear → Direction 0 (fly-over)
     *
     * A segment is considered "clear" if 80% of laser points are beyond 3.0m.
     *
     * @param msg Laser scan message with range data
     *
     * @note Called automatically 1 second after obstacle detection
     * @note Publishes avoidance direction to mission node
     * @see check_segment_clear()
     */
    void scan_for_clear_path(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        size_t ranges_size = msg->ranges.size();

        // Check 30° segments: right (270°) and left (90°)
        int right_center = ranges_size * 3 / 4;
        int left_center = ranges_size / 4;
        int segment_width = ranges_size / 12; // ±15°

        bool right_clear = check_segment_clear(msg, right_center, segment_width, 3.0);
        bool left_clear = check_segment_clear(msg, left_center, segment_width, 3.0);

        int avoidance_direction = 0;
        if (right_clear) {
            avoidance_direction = 1; // Fly-around right
            RCLCPP_INFO(this->get_logger(), "RIGHT side clear - FLY-AROUND");
        } else if (left_clear) {
            avoidance_direction = 2; // Fly-around left
            RCLCPP_INFO(this->get_logger(), "LEFT side clear - FLY-AROUND");
        } else {
            avoidance_direction = 0; // Fly-over
            RCLCPP_WARN(this->get_logger(), "No clear path - FLY-OVER");
        }

        auto direction_msg = std_msgs::msg::Int32();
        direction_msg.data = avoidance_direction;
        avoidance_direction_pub_->publish(direction_msg);

        control_state_ = ControlState::OBSTACLE_STOP;
    }

    bool check_segment_clear(const sensor_msgs::msg::LaserScan::SharedPtr msg,
                             int center_idx, int half_width, double max_distance) {
        size_t ranges_size = msg->ranges.size();
        int clear_count = 0;
        int total_count = 0;

        for (int i = -half_width; i <= half_width; ++i) {
            int idx = (center_idx + i + ranges_size) % ranges_size;
            float range = msg->ranges[idx];

            if (std::isnan(range) || std::isinf(range)) {
                clear_count++;
                total_count++;
                continue;
            }

            if (range > max_distance || range < 0.2) {
                clear_count++;
            }
            total_count++;
        }

        return static_cast<double>(clear_count) / std::max(total_count, 1) >= 0.8;
    }

    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

    void print_status() {
        if (!has_target_) {
            return;
        }

        auto now = this->now();
        if ((now - last_status_print_).seconds() < 5.0) {
            return;
        }
        last_status_print_ = now;

        double dx = target_position_.x - current_position_.x;
        double dy = target_position_.y - current_position_.y;
        double dz = target_position_.z - current_position_.z;
        double distance = std::sqrt(dx*dx + dy*dy);

        std::string state_str;
        switch (control_state_) {
            case ControlState::IDLE: state_str = "IDLE"; break;
            case ControlState::CLIMBING: state_str = "CLIMBING"; break;
            case ControlState::TURNING: state_str = "TURNING"; break;
            case ControlState::MOVING: state_str = "MOVING"; break;
            case ControlState::OBSTACLE_SCANNING: state_str = "SCANNING"; break;
            case ControlState::OBSTACLE_STOP: state_str = "OBSTACLE_STOP"; break;
            case ControlState::AVOIDANCE_MANEUVER: state_str = "AVOIDANCE"; break;
            case ControlState::OBSTACLE_DETECTED: state_str = "DETECTED"; break;
            case ControlState::EMERGENCY_LANDING: state_str = "EMERGENCY_LANDING"; break;
        }

        RCLCPP_INFO(this->get_logger(),
            "Status: %s | Pos (%.2f, %.2f, %.2f) | Dist %.2fm | Alt error %.2fm",
            state_str.c_str(),
            current_position_.x, current_position_.y, current_position_.z,
            distance, dz);
    }

    
    // UI CONTROL CALLBACKS

    void pause_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && !is_paused_) {
            is_paused_ = true;
            RCLCPP_INFO(this->get_logger(), "Navigation PAUSED from UI");
        }
    }

    void resume_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && is_paused_) {
            is_paused_ = false;
            RCLCPP_INFO(this->get_logger(), "Navigation RESUMED from UI");
        }
    }

    void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            control_state_ = ControlState::EMERGENCY_LANDING;
            has_target_ = false;
            is_paused_ = false;
            RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP - Initiating safe landing!");
        }
    }

    void control_loop() {
        // Check if paused
        if (is_paused_) {
            auto cmd = geometry_msgs::msg::Twist(); // Zero velocity
            cmd_pub_->publish(cmd);
            return;
        }

        auto cmd = geometry_msgs::msg::Twist();
        
        if (!has_target_ && control_state_ != ControlState::EMERGENCY_LANDING) {
            // No target, remain idle (emergency landing handled below even without a waypoint)
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
        if (control_state_ != ControlState::EMERGENCY_LANDING &&
            horizontal_distance <= position_tolerance_ && 
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
        
        // Handle obstacle holding states
        if (control_state_ == ControlState::OBSTACLE_STOP ||
            control_state_ == ControlState::OBSTACLE_SCANNING ||
            control_state_ == ControlState::OBSTACLE_DETECTED) {
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

            case ControlState::AVOIDANCE_MANEUVER:
                {
                    if (std::abs(altitude_error) > climbing_threshold_) {
                        cmd.linear.x = 0.0;
                        break;
                    }

                    if (horizontal_distance > 1.0 && yaw_error_degrees > turning_threshold_degrees_) {
                        cmd.linear.x = 0.0;
                        double turn_speed = 0.5;
                        double turn_gain = std::min(1.0, std::abs(yaw_error) / (M_PI / 4));
                        turn_gain = std::max(0.3, turn_gain);
                        cmd.angular.z = turn_gain * turn_speed * (yaw_error > 0 ? 1.0 : -1.0);
                    } else {
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

                        if (horizontal_distance > 1.0 && yaw_error_degrees > heading_tolerance_degrees_) {
                            double correction_gain = 0.15;
                            cmd.angular.z = yaw_error * correction_gain;
                            cmd.angular.z = std::max(-0.2, std::min(0.2, cmd.angular.z));
                        } else {
                            cmd.angular.z = 0.0;
                        }
                    }
                }
                break;
                
            case ControlState::EMERGENCY_LANDING:
                {
                    // Safe landing - descend slowly to ground
                    double landing_altitude = 0.1; // Land at near ground
                    double current_altitude = current_position_.z;

                    if (current_altitude <= landing_altitude) {
                        // Reached ground, stop everything
                        cmd.linear.x = 0.0;
                        cmd.linear.y = 0.0;
                        cmd.linear.z = 0.0;
                        cmd.angular.z = 0.0;
                        control_state_ = ControlState::IDLE;
                        RCLCPP_INFO(this->get_logger(), "Emergency landing complete. Drone on ground.");
                    } else {
                        // Descend slowly
                        cmd.linear.x = 0.0;
                        cmd.linear.y = 0.0;
                        cmd.linear.z = -0.3; // Slow descent at 0.3 m/s
                        cmd.angular.z = 0.0;

                        static int landing_log_counter = 0;
                        if (++landing_log_counter % 10 == 0) {
                            RCLCPP_WARN(this->get_logger(),
                                "EMERGENCY LANDING: Current altitude: %.2fm, descending...",
                                current_altitude);
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
        
        // Add constant to fight gravity (when gravity is enabled)
        // Set to 0.0 when gravity is disabled
        double gravity_compensation = 0.0; // No gravity compensation needed
        
        return climb_speed + gravity_compensation;
    }
    
    // Member variables
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr waypoint_reached_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_detected_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr avoidance_direction_pub_;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr avoidance_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr resume_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    struct Position {
        double x, y, z;
    };
    
    Position current_position_;
    Position target_position_;
    double current_yaw_;
    
    ControlState control_state_;
    bool has_target_;
    bool obstacle_detected_;
    bool emergency_triggered_;
    bool is_paused_;
    bool in_avoidance_mode_;

    double position_tolerance_;
    double altitude_tolerance_;
    double climbing_threshold_;
    double turning_threshold_degrees_;
    double heading_tolerance_degrees_;
    double normal_obstacle_threshold_;
    double avoidance_obstacle_threshold_;
    double emergency_obstacle_threshold_;

    rclcpp::Time scan_start_time_;
    rclcpp::Time last_status_print_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
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
