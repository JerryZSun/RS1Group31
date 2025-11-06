#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <chrono>

enum class ControlState {
    IDLE,
    CLIMBING,
    TURNING,
    MOVING,
    OBSTACLE_SCANNING,
    OBSTACLE_STOP,
    AVOIDANCE_MANEUVER
};

class Navigation : public rclcpp::Node {
public:
    Navigation() : Node("navigation"),
                   control_state_(ControlState::IDLE),
                   has_target_(false),
                   obstacle_detected_(false),
                   in_avoidance_mode_(false),
                   emergency_triggered_(false),
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
            "/odometry/filtered", 10,
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
        
        // Control timer - 10Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Navigation::control_loop, this)
        );
        
        // Status timer - 0.2Hz (every 5 seconds)
        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5000),
            std::bind(&Navigation::print_status, this)
        );
        
        // Initialize parameters
        current_position_ = {0.0, 0.0, 0.0};
        target_position_ = {0.0, 0.0, 0.0};
        current_yaw_ = 0.0;
        
        position_tolerance_ = 0.5;
        altitude_tolerance_ = 0.3;
        climbing_threshold_ = 2.0;
        turning_threshold_degrees_ = 30.0;
        heading_tolerance_degrees_ = 5.0;
        
        // Obstacle detection parameters
        normal_obstacle_threshold_ = 2.0;
        avoidance_obstacle_threshold_ = 1.5;
        emergency_obstacle_threshold_ = 0.5;
        
        RCLCPP_INFO(this->get_logger(), "Navigation Node Initialized");
        RCLCPP_INFO(this->get_logger(), "EMERGENCY STOP: Always active at 0.5m");
        RCLCPP_INFO(this->get_logger(), "Detection thresholds - Normal: 2.0m, Avoidance: 1.5m, Emergency: 0.5m");
    }

private:
    void waypoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        target_position_.x = msg->pose.position.x;
        target_position_.y = msg->pose.position.y;
        target_position_.z = msg->pose.position.z;
        
        has_target_ = true;
        
        // Reset flags for new waypoint (unless emergency is active)
        if (!emergency_triggered_) {
            if (control_state_ != ControlState::AVOIDANCE_MANEUVER && 
                control_state_ != ControlState::OBSTACLE_STOP) {
                obstacle_detected_ = false;
            }
        }
        
        // If in avoidance mode, stay in AVOIDANCE_MANEUVER state
        if (in_avoidance_mode_ && !emergency_triggered_) {
            control_state_ = ControlState::AVOIDANCE_MANEUVER;
            RCLCPP_INFO(this->get_logger(), 
                "Avoidance waypoint: (%.2f, %.2f, %.2f) - Normal detection disabled, Emergency still active", 
                target_position_.x, target_position_.y, target_position_.z);
        } else if (emergency_triggered_) {
            // Emergency fly-over waypoint - keep emergency mode
            control_state_ = ControlState::AVOIDANCE_MANEUVER;
            RCLCPP_INFO(this->get_logger(), 
                "Emergency fly-over waypoint: (%.2f, %.2f, %.2f)", 
                target_position_.x, target_position_.y, target_position_.z);
        } else {
            // Determine initial state
            double altitude_error = std::abs(target_position_.z - current_position_.z);
            if (altitude_error > climbing_threshold_) {
                control_state_ = ControlState::CLIMBING;
            } else {
                control_state_ = ControlState::TURNING;
            }
            
            RCLCPP_INFO(this->get_logger(), 
                "New waypoint: (%.2f, %.2f, %.2f)", 
                target_position_.x, target_position_.y, target_position_.z);
        }
    }
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_position_.x = msg->pose.pose.position.x;
        current_position_.y = msg->pose.pose.position.y;
        current_position_.z = msg->pose.pose.position.z;
        
        auto& q = msg->pose.pose.orientation;
        current_yaw_ = std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        );
    }
    
    void avoidance_mode_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        in_avoidance_mode_ = msg->data;
        
        // Emergency resets avoidance mode
        if (emergency_triggered_ && msg->data) {
            RCLCPP_WARN(this->get_logger(), "Emergency active - overriding avoidance mode");
            return;
        }
        
        if (in_avoidance_mode_) {
            RCLCPP_INFO(this->get_logger(), "Avoidance mode ACTIVE - Normal detection DISABLED, Emergency ACTIVE");
        } else {
            RCLCPP_INFO(this->get_logger(), "Avoidance mode INACTIVE - Normal detection active");
            emergency_triggered_ = false; // Reset emergency when exiting avoidance
        }
    }
    
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = msg;
        
        if (!has_target_) {
            return;
        }
        
        // ========================================
        // EMERGENCY CHECK - ALWAYS ACTIVE (0.5m)
        // ========================================
        if (check_emergency_obstacle(msg)) {
            // Only trigger once per obstacle
            if (control_state_ != ControlState::OBSTACLE_STOP || !emergency_triggered_) {
                emergency_triggered_ = true;
                obstacle_detected_ = true;
                control_state_ = ControlState::OBSTACLE_STOP;
                
                RCLCPP_ERROR(this->get_logger(), 
                    "🚨 EMERGENCY: Obstacle within 0.5m - Immediate FLY-OVER initiated! 🚨");
                
                // Immediately publish direction = 0 to trigger fly-over
                auto direction_msg = std_msgs::msg::Int32();
                direction_msg.data = 0;
                avoidance_direction_pub_->publish(direction_msg);
                
                // Publish obstacle detected
                auto obstacle_msg = std_msgs::msg::Bool();
                obstacle_msg.data = true;
                obstacle_detected_pub_->publish(obstacle_msg);
            }
            return; // Stop all other processing
        }
        
        // Skip normal detection during avoidance or emergency
        if (control_state_ == ControlState::AVOIDANCE_MANEUVER || emergency_triggered_) {
            return;
        }
        
        // Handle scanning state
        if (control_state_ == ControlState::OBSTACLE_SCANNING) {
            auto current_time = this->now();
            auto elapsed = (current_time - scan_start_time_).seconds();
            
            if (elapsed >= 1.0) {
                scan_for_clear_path(msg);
            }
            return;
        }
        
        // Normal obstacle detection (only when MOVING)
        if (control_state_ != ControlState::MOVING) {
            return;
        }
        
        // Check alignment with target
        double dx = target_position_.x - current_position_.x;
        double dy = target_position_.y - current_position_.y;
        double target_yaw = std::atan2(dy, dx);
        double yaw_error = normalize_angle(target_yaw - current_yaw_);
        double yaw_error_degrees = std::abs(yaw_error) * 180.0 / M_PI;
        
        // Only check if aligned
        if (yaw_error_degrees > 15.0) {
            return;
        }
        
        double obstacle_threshold = normal_obstacle_threshold_;
        int cone_range = msg->ranges.size() / 9; // ±20°
        
        size_t ranges_size = msg->ranges.size();
        bool obstacle_in_front = false;
        float closest_range = std::numeric_limits<float>::max();
        int detection_count = 0;
        
        for (int i = -cone_range; i <= cone_range; ++i) {
            int idx = (i + ranges_size) % ranges_size;
            float range = msg->ranges[idx];
            
            if (std::isnan(range) || std::isinf(range) || range < 0.2) {
                continue;
            }
            
            if (range < obstacle_threshold && range > 0.2) {
                detection_count++;
                closest_range = std::min(closest_range, range);
            }
        }
        
        if (detection_count >= 5) {
            obstacle_in_front = true;
        }
        
        // Handle obstacle detection
        if (obstacle_in_front && !obstacle_detected_) {
            obstacle_detected_ = true;
            control_state_ = ControlState::OBSTACLE_SCANNING;
            scan_start_time_ = this->now();
            
            RCLCPP_WARN(this->get_logger(), 
                "OBSTACLE DETECTED at %.2fm - Starting scan protocol", closest_range);
            
            auto obstacle_msg = std_msgs::msg::Bool();
            obstacle_msg.data = true;
            obstacle_detected_pub_->publish(obstacle_msg);
        }
    }
    
    bool check_emergency_obstacle(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        size_t ranges_size = msg->ranges.size();
        int front_range = ranges_size / 9; // ±20°
        
        int detection_count = 0;
        float closest_range = std::numeric_limits<float>::max();
        
        for (int i = -front_range; i <= front_range; ++i) {
            int idx = (i + ranges_size) % ranges_size;
            float range = msg->ranges[idx];
            
            if (std::isnan(range) || std::isinf(range) || range < 0.2) {
                continue;
            }
            
            if (range < emergency_obstacle_threshold_ && range > 0.2) {
                detection_count++;
                closest_range = std::min(closest_range, range);
            }
        }
        
        if (detection_count >= 3) {
            RCLCPP_ERROR(this->get_logger(), 
                "Emergency: %d points at %.2fm", detection_count, closest_range);
            return true;
        }
        
        return false;
    }
    
    void scan_for_clear_path(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        size_t ranges_size = msg->ranges.size();
        
        // Check 30° segments: right (±15°) and left (±15°)
        int right_center = ranges_size * 3 / 4; // 270°
        int segment_width = ranges_size / 12; // ±15°
        int left_center = ranges_size / 4; // 90°
        
        bool right_clear = check_segment_clear(msg, right_center, segment_width, 3.0);
        bool left_clear = check_segment_clear(msg, left_center, segment_width, 3.0);
        
        int avoidance_direction = 0;
        if (right_clear) {
            avoidance_direction = 1;
            RCLCPP_INFO(this->get_logger(), "Scan complete: RIGHT side clear - Initiating FLY-AROUND");
        } else if (left_clear) {
            avoidance_direction = 2;
            RCLCPP_INFO(this->get_logger(), "Scan complete: LEFT side clear - Initiating FLY-AROUND");
        } else {
            avoidance_direction = 0;
            RCLCPP_WARN(this->get_logger(), "Scan complete: No clear path - Defaulting to FLY-OVER");
        }
        
        // Publish direction
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
        
        return (static_cast<double>(clear_count) / total_count) >= 0.8;
    }
    
    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
    
    void print_status() {
        if (!has_target_) return;
        
        // Only print if enough time has passed (5 seconds)
        auto current_time = this->now();
        auto elapsed = (current_time - last_status_print_).seconds();
        
        if (elapsed < 5.0) {
            return;
        }
        
        last_status_print_ = current_time;
        
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
            case ControlState::AVOIDANCE_MANEUVER: state_str = emergency_triggered_ ? "EMERGENCY" : "AVOIDANCE"; break;
        }
        
        RCLCPP_INFO(this->get_logger(), 
            "Status: %s | Pos: (%.2f, %.2f, %.2f) | Distance: %.2fm | Alt error: %.2fm",
            state_str.c_str(),
            current_position_.x, current_position_.y, current_position_.z,
            distance, dz);
    }
    
    void control_loop() {
        auto cmd = geometry_msgs::msg::Twist();
        
        if (!has_target_) {
            cmd_pub_->publish(cmd);
            return;
        }
        
        // Calculate errors
        double dx = target_position_.x - current_position_.x;
        double dy = target_position_.y - current_position_.y;
        double dz = target_position_.z - current_position_.z;
        double horizontal_distance = std::sqrt(dx*dx + dy*dy);
        double altitude_error = dz;
        
        // Check if waypoint reached
        if (horizontal_distance <= position_tolerance_ && 
            std::abs(altitude_error) <= altitude_tolerance_) {
            RCLCPP_INFO(this->get_logger(), 
                "Waypoint reached: (%.2f, %.2f, %.2f)", 
                current_position_.x, current_position_.y, current_position_.z);
            
            auto reached_msg = std_msgs::msg::Bool();
            reached_msg.data = true;
            waypoint_reached_pub_->publish(reached_msg);
            
            has_target_ = false;
            control_state_ = ControlState::IDLE;
            cmd_pub_->publish(cmd);
            return;
        }
        
        // Handle special states
        if (control_state_ == ControlState::OBSTACLE_STOP || 
            control_state_ == ControlState::OBSTACLE_SCANNING) {
            cmd.linear.z = calculate_altitude_control(altitude_error);
            cmd_pub_->publish(cmd);
            return;
        }
        
        // Calculate heading
        double yaw_error = 0.0;
        double yaw_error_degrees = 0.0;
        
        if (horizontal_distance > 1.0) {
            double target_yaw = std::atan2(dy, dx);
            yaw_error = normalize_angle(target_yaw - current_yaw_);
            yaw_error_degrees = std::abs(yaw_error) * 180.0 / M_PI;
        }
        
        cmd.linear.z = calculate_altitude_control(altitude_error);
        
        // Check if need to prioritize climbing
        if (std::abs(altitude_error) > climbing_threshold_ && 
            control_state_ != ControlState::CLIMBING &&
            control_state_ != ControlState::AVOIDANCE_MANEUVER) {
            control_state_ = ControlState::CLIMBING;
        }
        
        // State machine
        switch (control_state_) {
            case ControlState::CLIMBING:
                if (std::abs(altitude_error) <= altitude_tolerance_) {
                    control_state_ = ControlState::TURNING;
                } else {
                    cmd.linear.x = 0.0;
                    cmd.angular.z = 0.0;
                }
                break;
                
            case ControlState::TURNING:
                if (yaw_error_degrees <= heading_tolerance_degrees_) {
                    control_state_ = ControlState::MOVING;
                } else {
                    double turn_speed = 0.5;
                    double turn_gain = std::min(1.0, std::abs(yaw_error) / (M_PI / 4));
                    turn_gain = std::max(0.3, turn_gain);
                    
                    cmd.angular.z = turn_gain * turn_speed * (yaw_error > 0 ? 1.0 : -1.0);
                    cmd.linear.x = 0.0;
                }
                break;
                
            case ControlState::MOVING:
                if (std::abs(altitude_error) > climbing_threshold_) {
                    control_state_ = ControlState::CLIMBING;
                    cmd.linear.x = 0.0;
                    break;
                }
                
                if (horizontal_distance > 1.0 && yaw_error_degrees > turning_threshold_degrees_) {
                    control_state_ = ControlState::TURNING;
                    cmd.linear.x = 0.0;
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
                break;
                
            case ControlState::AVOIDANCE_MANEUVER:
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
                break;
                
            default:
                break;
        }
        
        cmd_pub_->publish(cmd);
    }
    
    double calculate_altitude_control(double altitude_error) {
        double kp_altitude = 0.3;
        double max_climb_speed = 0.5;
        
        double climb_speed = kp_altitude * altitude_error;
        climb_speed = std::max(-max_climb_speed, std::min(max_climb_speed, climb_speed));
        
        return climb_speed;
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
    bool in_avoidance_mode_;
    bool emergency_triggered_;
    
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