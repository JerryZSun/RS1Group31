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
    WAITING_FOR_ODOM,
    CLIMBING,
    TURNING,
    MOVING
};

class Mission : public rclcpp::Node {
public:
    Mission() : Node("mission"), 
                current_waypoint_index_(0), 
                goal_reached_(false),
                control_state_(ControlState::WAITING_FOR_ODOM),
                odom_received_(false),
                raw_odom_received_(false) {
        
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Subscribe to filtered odometry for X, Y, yaw
        odom_filtered_sub_ = this->create_subscription<nav_msgs::msg/Odometry>(
            "/odometry/filtered", 10,
            std::bind(&Mission::odom_filtered_callback, this, std::placeholders::_1)
        );
        
        // Subscribe to RAW Gazebo odometry for Z (since 2D SLAM/robot_loc don't track Z)
        odom_raw_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry", 10,
            std::bind(&Mission::odom_raw_callback, this, std::placeholders::_1)
        );
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Mission::control_loop, this)
        );
        
        waypoints_ = {
            {0, 0, 7},
            {9.5, -12, 7},
            {9.5, -12, 1.25},
            {7, -9.5, 1.25},
            {4, -6.5, 1.25},
            {2, -4.5, 1.25},
            {0, -1.5, 1.25},
            {3, 2.5, 1.25},
            {4, 4, 1.25},
            {6, 6, 1.25},
            {8, 8.5, 1.25},
            {11, 10.5, 1.25},
        };
        
        current_position_ = {0.0, 0.0, 0.0};
        current_yaw_ = 0.0;
        position_tolerance_ = 0.5;
        altitude_tolerance_ = 0.25;
        turning_threshold_degrees_ = 30.0;
        heading_tolerance_degrees_ = 5.0;
        
        RCLCPP_INFO(this->get_logger(), "Mission initialized with %zu waypoints", waypoints_.size());
        RCLCPP_INFO(this->get_logger(), "Using HYBRID odometry: X/Y from filtered, Z from raw Gazebo");
        RCLCPP_INFO(this->get_logger(), "Waiting for odometry data...");
    }

private:
    
    void odom_filtered_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Get X, Y from filtered odometry (SLAM/robot_localization)
        current_position_.x = msg->pose.pose.position.x;
        current_position_.y = msg->pose.pose.position.y;
        
        // Extract yaw from quaternion
        auto& q = msg->pose.pose.orientation;
        current_yaw_ = std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        );
        
        if (!odom_received_) {
            odom_received_ = true;
        }
        
        check_ready_to_start();
    }
    
    void odom_raw_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Get Z from raw Gazebo odometry (ground truth altitude)
        current_position_.z = msg->pose.pose.position.z;
        
        if (!raw_odom_received_) {
            raw_odom_received_ = true;
        }
        
        check_ready_to_start();
    }
    
    void check_ready_to_start() {
        static bool started = false;
        if (!started && odom_received_ && raw_odom_received_) {
            started = true;
            RCLCPP_INFO(this->get_logger(), 
                "Odometry received! Starting position: (%.2f, %.2f, %.2f)", 
                current_position_.x, current_position_.y, current_position_.z);
            RCLCPP_INFO(this->get_logger(), 
                "First waypoint: (%.1f, %.1f, %.1f)", 
                waypoints_[0].x, waypoints_[0].y, waypoints_[0].z);
            control_state_ = ControlState::CLIMBING;
        }
    }
    
    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
    
    void control_loop() {
        auto cmd = geometry_msgs::msg::Twist();
        
        switch (control_state_) {
            case ControlState::WAITING_FOR_ODOM:
                cmd.linear.x = 0.0;
                cmd.linear.y = 0.0;
                cmd.linear.z = 0.0;
                cmd.angular.z = 0.0;
                break;
                
            case ControlState::CLIMBING:
                handle_climbing(cmd);
                break;
                
            case ControlState::TURNING:
                handle_turning(cmd);
                break;
                
            case ControlState::MOVING:
                handle_moving(cmd);
                break;
        }
        
        cmd_pub_->publish(cmd);
    }
    
    void handle_climbing(geometry_msgs::msg::Twist& cmd) {
        if (current_waypoint_index_ >= waypoints_.size()) {
            if (!goal_reached_) {
                RCLCPP_INFO(this->get_logger(), "All waypoints reached! Mission complete.");
                goal_reached_ = true;
            }
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.linear.z = 0.0;
            cmd.angular.z = 0.0;
            return;
        }
        
        const Waypoint& target = waypoints_[current_waypoint_index_];
        double target_altitude = target.z;
        double altitude_error = target_altitude - current_position_.z;
        
        if (std::abs(altitude_error) <= altitude_tolerance_) {
            control_state_ = ControlState::TURNING;
            RCLCPP_INFO(this->get_logger(), 
                "Target altitude %.2f m reached! Current: %.2f m. Switching to TURNING.", 
                target_altitude, current_position_.z);
            
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.linear.z = 0.0;
            cmd.angular.z = 0.0;
            return;
        }
        
        double min_climb_speed = 0.05;
        double max_climb_speed = 0.8;
        double climb_gain = 1.5;
        double climb_speed = climb_gain * altitude_error;
        
        climb_speed = std::max(-max_climb_speed, std::min(max_climb_speed, climb_speed));
        
        if (std::abs(climb_speed) > 0.01) {
            if (std::abs(climb_speed) < min_climb_speed) {
                climb_speed = (climb_speed > 0) ? min_climb_speed : -min_climb_speed;
            }
        }
        
        if (std::abs(altitude_error) < 0.5) {
            climb_speed *= 0.6;
        }
        
        cmd.linear.z = climb_speed;
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = 0.0;
        
        static int climb_log_counter = 0;
        if (++climb_log_counter % 5 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                "CLIMBING: Pos(%.2f, %.2f, %.2f) -> Target: %.2f m, Error: %.2f m, Speed: %.2f m/s", 
                current_position_.x, current_position_.y, current_position_.z,
                target_altitude, altitude_error, climb_speed);
        }
    }
    
    void handle_turning(geometry_msgs::msg::Twist& cmd) {
        if (current_waypoint_index_ >= waypoints_.size()) {
            if (!goal_reached_) {
                RCLCPP_INFO(this->get_logger(), "All waypoints reached! Mission complete.");
                goal_reached_ = true;
            }
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.linear.z = 0.0;
            cmd.angular.z = 0.0;
            return;
        }
        
        const Waypoint& target = waypoints_[current_waypoint_index_];
        
        double dx = target.x - current_position_.x;
        double dy = target.y - current_position_.y;
        double horizontal_distance = std::sqrt(dx*dx + dy*dy);
        
        double yaw_error = 0.0;
        double yaw_error_degrees = 0.0;
        
        if (horizontal_distance > 0.5) {
            double target_yaw = std::atan2(dy, dx);
            yaw_error = normalize_angle(target_yaw - current_yaw_);
            yaw_error_degrees = std::abs(yaw_error) * 180.0 / M_PI;
        } else {
            yaw_error_degrees = 0.0;
        }
        
        if (yaw_error_degrees <= heading_tolerance_degrees_) {
            control_state_ = ControlState::MOVING;
            RCLCPP_INFO(this->get_logger(), 
                "Heading aligned (%.1f°), switching to MOVING", yaw_error_degrees);
        } else {
            double turn_speed = 0.5;
            double turn_gain = std::min(1.0, std::abs(yaw_error) / (M_PI / 4));
            turn_gain = std::max(0.3, turn_gain);
            
            cmd.angular.z = turn_gain * turn_speed * (yaw_error > 0 ? 1.0 : -1.0);
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            
            double altitude_error = target.z - current_position_.z;
            if (std::abs(altitude_error) > altitude_tolerance_) {
                double vertical_correction = std::max(-0.2, std::min(0.2, altitude_error * 0.8));
                cmd.linear.z = vertical_correction;
            } else {
                cmd.linear.z = 0.0;
            }
            
            static int turn_log_counter = 0;
            if (++turn_log_counter % 10 == 0) {
                RCLCPP_INFO(this->get_logger(), 
                    "TURNING: Pos(%.2f, %.2f, %.2f) Target(%.1f, %.1f, %.1f) Yaw error: %.1f°", 
                    current_position_.x, current_position_.y, current_position_.z,
                    target.x, target.y, target.z, yaw_error_degrees);
            }
        }
    }
    
    void handle_moving(geometry_msgs::msg::Twist& cmd) {
        if (current_waypoint_index_ >= waypoints_.size()) {
            if (!goal_reached_) {
                RCLCPP_INFO(this->get_logger(), "All waypoints reached! Mission complete.");
                goal_reached_ = true;
            }
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.linear.z = 0.0;
            cmd.angular.z = 0.0;
            return;
        }
        
        const Waypoint& target = waypoints_[current_waypoint_index_];
        
        double dx = target.x - current_position_.x;
        double dy = target.y - current_position_.y;
        double dz = target.z - current_position_.z;
        double horizontal_distance = std::sqrt(dx*dx + dy*dy);
        
        if (horizontal_distance <= position_tolerance_ && 
            std::abs(dz) <= altitude_tolerance_) {
            RCLCPP_INFO(this->get_logger(), 
                "Waypoint %zu reached at (%.2f, %.2f, %.2f)!", 
                current_waypoint_index_ + 1,
                current_position_.x, current_position_.y, current_position_.z);
            current_waypoint_index_++;
            
            if (current_waypoint_index_ < waypoints_.size()) {
                const Waypoint& next_target = waypoints_[current_waypoint_index_];
                double next_altitude_error = std::abs(next_target.z - current_position_.z);
                
                if (next_altitude_error > altitude_tolerance_) {
                    control_state_ = ControlState::CLIMBING;
                    RCLCPP_INFO(this->get_logger(), 
                        "Next waypoint altitude change: %.2f -> %.2f m. CLIMBING.", 
                        current_position_.z, next_target.z);
                } else {
                    control_state_ = ControlState::TURNING;
                }
            }
            
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.linear.z = 0.0;
            cmd.angular.z = 0.0;
            return;
        }
        
        double yaw_error = 0.0;
        double yaw_error_degrees = 0.0;
        
        if (horizontal_distance > 1.0) {
            double target_yaw = std::atan2(dy, dx);
            yaw_error = normalize_angle(target_yaw - current_yaw_);
            yaw_error_degrees = std::abs(yaw_error) * 180.0 / M_PI;
        }
        
        if (horizontal_distance > 1.0 && yaw_error_degrees > turning_threshold_degrees_) {
            control_state_ = ControlState::TURNING;
            RCLCPP_INFO(this->get_logger(), 
                "Heading deviation %.1f°, switching to TURNING", yaw_error_degrees);
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.linear.z = 0.0;
            cmd.angular.z = 0.0;
            return;
        }
        
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
        
        double forward_speed = base_speed * speed_factor;
        
        cmd.linear.x = forward_speed;
        cmd.linear.y = 0.0;
        
        double altitude_error = target.z - current_position_.z;
        if (std::abs(altitude_error) > altitude_tolerance_) {
            double vertical_gain = 1.2;
            double vertical_speed = vertical_gain * altitude_error;
            
            double max_vertical_speed = 0.5;
            vertical_speed = std::max(-max_vertical_speed, 
                                     std::min(max_vertical_speed, vertical_speed));
            
            cmd.linear.z = vertical_speed;
        } else {
            cmd.linear.z = 0.0;
        }
        
        if (horizontal_distance > 1.0 && yaw_error_degrees > heading_tolerance_degrees_) {
            double correction_gain = 0.15;
            cmd.angular.z = yaw_error * correction_gain;
            
            double max_correction = 0.2;
            cmd.angular.z = std::max(-max_correction, 
                                    std::min(max_correction, cmd.angular.z));
        } else {
            cmd.angular.z = 0.0;
        }
        
        static int move_log_counter = 0;
        if (++move_log_counter % 20 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                "MOVING: Pos(%.2f,%.2f,%.2f) -> WP%zu(%.1f,%.1f,%.1f) Dist:%.2fm Alt:%.2fm", 
                current_position_.x, current_position_.y, current_position_.z,
                current_waypoint_index_ + 1, 
                target.x, target.y, target.z, 
                horizontal_distance, altitude_error);
        }
    }
    
    // Member variables
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_filtered_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_raw_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::vector<Waypoint> waypoints_;
    size_t current_waypoint_index_;
    bool goal_reached_;
    ControlState control_state_;
    bool odom_received_;
    bool raw_odom_received_;
    
    Waypoint current_position_;
    double current_yaw_;
    
    double position_tolerance_;
    double altitude_tolerance_;
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