#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "trail_patrol_drone/msg/obstacle_avoidance_request.hpp"
#include <cmath>
#include <chrono>
#include <vector>

enum class ControlState {
    IDLE,
    CLIMBING,
    TURNING,
    MOVING,
    OBSTACLE_DETECTED,
    AVOIDANCE_MODE,
    EMERGENCY_STOP
};

class Navigation : public rclcpp::Node {
public:
    Navigation() : Node("navigation"),
                   control_state_(ControlState::IDLE),
                   has_target_(false),
                   obstacle_detected_(false),
                   in_avoidance_mode_(false),
                   emergency_stop_(false),
                   status_counter_(0) {
        
        // Publishers
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        waypoint_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/navigation/waypoint_reached", 10);
        avoidance_request_pub_ = this->create_publisher<trail_patrol_drone::msg::ObstacleAvoidanceRequest>(
            "/navigation/obstacle_avoidance_request", 10);
        
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
        
        // Status timer - 2Hz
        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
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
        normal_obstacle_threshold_ = 2.0;
        avoidance_obstacle_threshold_ = 1.0;
        avoidance_clear_distance_ = 3.0;
        
        RCLCPP_INFO(this->get_logger(), "=== Navigation Node Initialized ===");
        RCLCPP_INFO(this->get_logger(), "Normal mode: 2m obstacle detection");
        RCLCPP_INFO(this->get_logger(), "Avoidance mode: 1m emergency stop only");
    }

private:
    void waypoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        target_position_.x = msg->pose.position.x;
        target_position_.y = msg->pose.position.y;
        target_position_.z = msg->pose.position.z;
        
        has_target_ = true;
        
        // Reset obstacle detection if not in emergency
        if (control_state_ != ControlState::EMERGENCY_STOP) {
            obstacle_detected_ = false;
        }
        
        // Determine initial state
        double altitude_error = std::abs(target_position_.z - current_position_.z);
        if (altitude_error > climbing_threshold_) {
            control_state_ = ControlState::CLIMBING;
        } else {
            control_state_ = ControlState::TURNING;
        }
        
        RCLCPP_INFO(this->get_logger(), 
            "New waypoint received: (%.2f, %.2f, %.2f)", 
            target_position_.x, target_position_.y, target_position_.z);
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
        bool new_mode = msg->data;
        if (new_mode != in_avoidance_mode_) {
            in_avoidance_mode_ = new_mode;
            if (in_avoidance_mode_) {
                RCLCPP_WARN(this->get_logger(), "Entering AVOIDANCE MODE: 1m emergency threshold");
                control_state_ = ControlState::AVOIDANCE_MODE;
            } else {
                RCLCPP_INFO(this->get_logger(), "Exiting AVOIDANCE MODE: Normal 2m detection");
            }
        }
    }
    
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Skip if no target or in emergency stop
        if (!has_target_ || control_state_ == ControlState::EMERGENCY_STOP) {
            return;
        }
        
        // Different thresholds based on mode
        double threshold = in_avoidance_mode_ ? avoidance_obstacle_threshold_ : normal_obstacle_threshold_;
        
        // Only check when MOVING or in AVOIDANCE_MODE
        if (control_state_ != ControlState::MOVING && control_state_ != ControlState::AVOIDANCE_MODE) {
            return;
        }
        
        // Check alignment
        double dx = target_position_.x - current_position_.x;
        double dy = target_position_.y - current_position_.y;
        double target_yaw = std::atan2(dy, dx);
        double yaw_error = normalize_angle(target_yaw - current_yaw_);
        double yaw_error_degrees = std::abs(yaw_error) * 180.0 / M_PI;
        
        // Only check if aligned
        if (yaw_error_degrees > 15.0) {
            return;
        }
        
        size_t ranges_size = msg->ranges.size();
        
        // Check forward cone (±20° = 40° total)
        int front_range = ranges_size / 9;
        
        bool obstacle_in_front = false;
        float closest_range = std::numeric_limits<float>::max();
        int detection_count = 0;
        
        for (int i = -front_range; i <= front_range; ++i) {
            int idx = (i + ranges_size) % ranges_size;
            float range = msg->ranges[idx];
            
            if (std::isnan(range) || std::isinf(range) || range < 0.2) {
                continue;
            }
            
            if (range < threshold && range > 0.2) {
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
            
            if (in_avoidance_mode_) {
                // Emergency stop in avoidance mode
                emergency_stop_ = true;
                control_state_ = ControlState::EMERGENCY_STOP;
                
                RCLCPP_ERROR(this->get_logger(), 
                    "╔════════════════════════════════════════════════════════════╗");
                RCLCPP_ERROR(this->get_logger(), 
                    "║ EMERGENCY STOP: Obstacle within 1m during avoidance!      ║");
                RCLCPP_ERROR(this->get_logger(), 
                    "║ Position: (%.2f, %.2f, %.2f)                         ║", 
                    current_position_.x, current_position_.y, current_position_.z);
                RCLCPP_ERROR(this->get_logger(), 
                    "║ Distance: %.2fm                                            ║", 
                    closest_range);
                RCLCPP_ERROR(this->get_logger(), 
                    "║ Requesting FLY-OVER protocol from Mission node            ║");
                RCLCPP_ERROR(this->get_logger(), 
                    "╚════════════════════════════════════════════════════════════╝");
                
                // Request fly-over by sending NO_PATH
                auto request = trail_patrol_drone::msg::ObstacleAvoidanceRequest();
                request.current_position.x = current_position_.x;
                request.current_position.y = current_position_.y;
                request.current_position.z = current_position_.z;
                request.best_clear_angle = -999.0;  // NO_PATH
                request.obstacle_distance = closest_range;
                avoidance_request_pub_->publish(request);
                
            } else {
                // Normal mode - find best direction
                control_state_ = ControlState::OBSTACLE_DETECTED;
                
                RCLCPP_WARN(this->get_logger(), 
                    "╔════════════════════════════════════════════════════════════╗");
                RCLCPP_WARN(this->get_logger(), 
                    "║ OBSTACLE DETECTED                                          ║");
                RCLCPP_WARN(this->get_logger(), 
                    "║ Position: (%.2f, %.2f, %.2f)                         ║", 
                    current_position_.x, current_position_.y, current_position_.z);
                RCLCPP_WARN(this->get_logger(), 
                    "║ Distance: %.2fm                                            ║", 
                    closest_range);
                RCLCPP_WARN(this->get_logger(), 
                    "╚════════════════════════════════════════════════════════════╝");
                
                // Find best direction
                double best_angle = find_best_clear_direction(msg);
                
                auto request = trail_patrol_drone::msg::ObstacleAvoidanceRequest();
                request.current_position.x = current_position_.x;
                request.current_position.y = current_position_.y;
                request.current_position.z = current_position_.z;
                request.best_clear_angle = best_angle;
                request.obstacle_distance = closest_range;
                avoidance_request_pub_->publish(request);
                
                if (best_angle != -999.0) {
                    RCLCPP_INFO(this->get_logger(), 
                        "Best clear direction found: %.1f° (%.2f rad)", 
                        best_angle * 180.0 / M_PI, best_angle);
                } else {
                    RCLCPP_ERROR(this->get_logger(), 
                        "NO CLEAR PATH found - requesting FLY-OVER protocol");
                }
            }
        }
        
        // Check if obstacle cleared in avoidance mode
        if (in_avoidance_mode_ && !obstacle_in_front && obstacle_detected_) {
            // Check if truly clear (no obstacle within 3m in front)
            bool truly_clear = true;
            for (int i = -front_range; i <= front_range; ++i) {
                int idx = (i + ranges_size) % ranges_size;
                float range = msg->ranges[idx];
                
                if (!std::isnan(range) && !std::isinf(range) && range > 0.2 && range < avoidance_clear_distance_) {
                    truly_clear = false;
                    break;
                }
            }
            
            if (truly_clear) {
                obstacle_detected_ = false;
                RCLCPP_INFO(this->get_logger(), "Obstacle cleared - path clear ahead");
            }
        }
    }
    
    double find_best_clear_direction(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        size_t ranges_size = msg->ranges.size();
        double angle_increment = 2.0 * M_PI / ranges_size;
        
        // Sector size: 10 degrees
        int sector_size = static_cast<int>(10.0 * M_PI / 180.0 / angle_increment);
        int num_sectors = ranges_size / sector_size;
        
        std::vector<bool> clear_sectors(ranges_size, true);
        
        // Mark angles where obstacles are <1m
        for (size_t i = 0; i < ranges_size; ++i) {
            float range = msg->ranges[i];
            if (!std::isnan(range) && !std::isinf(range) && range > 0.2 && range < 1.0) {
                clear_sectors[i] = false;
            }
        }
        
        // Find sectors with >=6m clear distance
        std::vector<double> valid_sector_angles;
        
        for (int sector = 0; sector < num_sectors; ++sector) {
            int start_idx = sector * sector_size;
            int end_idx = start_idx + sector_size;
            
            bool sector_valid = true;
            double min_range_in_sector = std::numeric_limits<double>::max();
            int clear_count = 0;
            
            for (int i = start_idx; i < end_idx && i < static_cast<int>(ranges_size); ++i) {
                if (!clear_sectors[i]) {
                    sector_valid = false;
                    break;
                }
                
                float range = msg->ranges[i];
                if (!std::isnan(range) && !std::isinf(range) && range > 0.2) {
                    min_range_in_sector = std::min(min_range_in_sector, static_cast<double>(range));
                    clear_count++;
                }
            }
            
            // Need at least 6m clear and some valid readings
            if (sector_valid && min_range_in_sector >= 6.0 && clear_count > 0) {
                double sector_angle = (start_idx + sector_size / 2) * angle_increment;
                valid_sector_angles.push_back(sector_angle);
            }
        }
        
        if (valid_sector_angles.empty()) {
            return -999.0;  // NO_PATH
        }
        
        // Find sector with minimum yaw difference to target
        double dx = target_position_.x - current_position_.x;
        double dy = target_position_.y - current_position_.y;
        double target_yaw = std::atan2(dy, dx);
        
        double best_angle = valid_sector_angles[0];
        double min_yaw_diff = std::abs(normalize_angle(valid_sector_angles[0] - target_yaw));
        
        for (double angle : valid_sector_angles) {
            double yaw_diff = std::abs(normalize_angle(angle - target_yaw));
            if (yaw_diff < min_yaw_diff) {
                min_yaw_diff = yaw_diff;
                best_angle = angle;
            }
        }
        
        return best_angle;
    }
    
    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
    
    void print_status() {
        if (!has_target_) return;
        
        double dx = target_position_.x - current_position_.x;
        double dy = target_position_.y - current_position_.y;
        double dz = target_position_.z - current_position_.z;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        double target_yaw = std::atan2(dy, dx);
        double yaw_error = normalize_angle(target_yaw - current_yaw_);
        double yaw_error_degrees = std::abs(yaw_error) * 180.0 / M_PI;
        
        std::string state_str;
        switch (control_state_) {
            case ControlState::IDLE: state_str = "IDLE"; break;
            case ControlState::CLIMBING: state_str = "CLIMBING"; break;
            case ControlState::TURNING: state_str = "TURNING"; break;
            case ControlState::MOVING: state_str = "MOVING"; break;
            case ControlState::OBSTACLE_DETECTED: state_str = "OBSTACLE_DETECTED"; break;
            case ControlState::AVOIDANCE_MODE: state_str = "AVOIDANCE_MODE"; break;
            case ControlState::EMERGENCY_STOP: state_str = "EMERGENCY_STOP"; break;
        }
        
        RCLCPP_INFO(this->get_logger(), 
            "STATUS | State: %s | Pos: (%.2f, %.2f, %.2f) | Target: (%.2f, %.2f, %.2f) | Dist: %.2fm | Alt Error: %.2fm | Yaw Error: %.1f°",
            state_str.c_str(),
            current_position_.x, current_position_.y, current_position_.z,
            target_position_.x, target_position_.y, target_position_.z,
            distance, dz, yaw_error_degrees);
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
        if (control_state_ == ControlState::OBSTACLE_DETECTED || 
            control_state_ == ControlState::EMERGENCY_STOP) {
            // Stop and maintain altitude
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
            control_state_ != ControlState::CLIMBING) {
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
                    control_state_ = in_avoidance_mode_ ? ControlState::AVOIDANCE_MODE : ControlState::MOVING;
                } else {
                    double turn_speed = 0.5;
                    double turn_gain = std::min(1.0, std::abs(yaw_error) / (M_PI / 4));
                    turn_gain = std::max(0.3, turn_gain);
                    
                    cmd.angular.z = turn_gain * turn_speed * (yaw_error > 0 ? 1.0 : -1.0);
                    cmd.linear.x = 0.0;
                }
                break;
                
            case ControlState::MOVING:
            case ControlState::AVOIDANCE_MODE:
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
    rclcpp::Publisher<trail_patrol_drone::msg::ObstacleAvoidanceRequest>::SharedPtr avoidance_request_pub_;
    
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
    bool emergency_stop_;
    int status_counter_;
    
    double position_tolerance_;
    double altitude_tolerance_;
    double climbing_threshold_;
    double turning_threshold_degrees_;
    double heading_tolerance_degrees_;
    double normal_obstacle_threshold_;
    double avoidance_obstacle_threshold_;
    double avoidance_clear_distance_;
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