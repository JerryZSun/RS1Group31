#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include <vector>
#include <cmath>

struct Waypoint {
    double x, y, z;
};

enum class MissionState {
    NORMAL,
    FLY_OVER_UP,
    FLY_OVER_ACROSS
};

class Mission : public rclcpp::Node {
public:
    Mission() : Node("mission"), 
                current_waypoint_index_(0), 
                mission_complete_(false),
                waypoint_reached_(false),
                first_waypoint_published_(false),
                mission_state_(MissionState::NORMAL),
                interrupted_waypoint_index_(0) {
        
        // Publishers
        waypoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mission/current_waypoint", 10);
        mission_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/mission/complete", 10);
        
        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,
            std::bind(&Mission::odom_callback, this, std::placeholders::_1)
        );
        
        waypoint_reached_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/navigation/waypoint_reached", 10,
            std::bind(&Mission::waypoint_reached_callback, this, std::placeholders::_1)
        );
        
        obstacle_detected_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/navigation/obstacle_detected", 10,
            std::bind(&Mission::obstacle_detected_callback, this, std::placeholders::_1)
        );
        
        // Timer for mission management
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&Mission::mission_loop, this)
        );
        
        // Initialize waypoints
        waypoints_ = {
            {0, 0, 0.1},           // Start
            {0, 0, 7},             // Climb up
            {9.5, -12, 7},         // Fly high to far corner
            {9.5, -12, 0.6},       // Descend to survey altitude
            {7, -9.5, 0.6},        // Survey waypoint
            {4, -8.5, 0.6},        // Survey waypoint
            {2, -7.5, 0.6},        // Survey waypoint
            {0, -1.5, 0.6},        // Survey waypoint
            {3, 2.5, 0.6},         // Survey waypoint
            {4, 3, 0.6},           // BEFORE OBSTACLE - drone should detect obstacle from here
            {8, 7.5, 0.6},         // AFTER OBSTACLE - (7, 6.25) obstacle is between these points
            {11, 10.5, 0.6},       // Continue survey
            {11, 10.5, 7},         // Climb back up
            {0, 0, 7},             // Return home high
            {0, 0, 0.1},           // Land
        };
        
        position_tolerance_ = 0.5;
        
        RCLCPP_INFO(this->get_logger(), "=== Mission Node Initialized ===");
        RCLCPP_INFO(this->get_logger(), "Total waypoints: %zu", waypoints_.size());
        RCLCPP_INFO(this->get_logger(), "Protocol: FLY-OVER only");
        RCLCPP_INFO(this->get_logger(), "Waiting for navigation node...");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_position_.x = msg->pose.pose.position.x;
        current_position_.y = msg->pose.pose.position.y;
        current_position_.z = msg->pose.pose.position.z;
    }
    
    void waypoint_reached_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            waypoint_reached_ = true;
        }
    }
    
    void obstacle_detected_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && mission_state_ == MissionState::NORMAL) {
            RCLCPP_ERROR(this->get_logger(), 
                "═══════════════════════════════════════════════════════════");
            RCLCPP_ERROR(this->get_logger(), 
                "OBSTACLE DETECTED - Activating FLY-OVER protocol");
            RCLCPP_ERROR(this->get_logger(), 
                "═══════════════════════════════════════════════════════════");
            
            activate_fly_over_protocol();
        }
    }
    
    void activate_fly_over_protocol() {
        // Save interrupted waypoint
        interrupted_waypoint_index_ = current_waypoint_index_;
        
        mission_state_ = MissionState::FLY_OVER_UP;
        
        // Generate first fly-over waypoint: straight up to z=7
        double up_x = current_position_.x;
        double up_y = current_position_.y;
        double up_z = 7.0;
        
        RCLCPP_INFO(this->get_logger(), 
            "FLY-OVER Step 1: Climbing to (%.2f, %.2f, %.2f)", 
            up_x, up_y, up_z);
        
        auto waypoint_msg = geometry_msgs::msg::PoseStamped();
        waypoint_msg.header.stamp = this->now();
        waypoint_msg.header.frame_id = "map";
        waypoint_msg.pose.position.x = up_x;
        waypoint_msg.pose.position.y = up_y;
        waypoint_msg.pose.position.z = up_z;
        waypoint_msg.pose.orientation.w = 1.0;
        
        waypoint_pub_->publish(waypoint_msg);
    }
    
    void mission_loop() {
        // Publish first waypoint once subscribers connect
        if (!first_waypoint_published_) {
            if (waypoint_pub_->get_subscription_count() > 0) {
                RCLCPP_INFO(this->get_logger(), "Navigation connected! Starting mission...");
                publish_current_waypoint();
                first_waypoint_published_ = true;
            }
            return;
        }
        
        // Check if mission complete
        if (current_waypoint_index_ >= waypoints_.size() && mission_state_ == MissionState::NORMAL) {
            if (!mission_complete_) {
                RCLCPP_INFO(this->get_logger(), 
                    "═══════════════════════════════════════════════════════════");
                RCLCPP_INFO(this->get_logger(), 
                    "MISSION COMPLETE: All %zu waypoints reached", 
                    waypoints_.size());
                RCLCPP_INFO(this->get_logger(), 
                    "═══════════════════════════════════════════════════════════");
                mission_complete_ = true;
                
                auto status_msg = std_msgs::msg::Bool();
                status_msg.data = true;
                mission_status_pub_->publish(status_msg);
            }
            return;
        }
        
        // Handle waypoint reached events
        if (waypoint_reached_) {
            waypoint_reached_ = false;
            
            switch (mission_state_) {
                case MissionState::NORMAL:
                    current_waypoint_index_++;
                    
                    if (current_waypoint_index_ < waypoints_.size()) {
                        RCLCPP_INFO(this->get_logger(), 
                            "Mission waypoint %zu/%zu reached, moving to next", 
                            current_waypoint_index_, waypoints_.size());
                        publish_current_waypoint();
                    }
                    break;
                    
                case MissionState::FLY_OVER_UP:
                    RCLCPP_INFO(this->get_logger(), 
                        "FLY-OVER climb complete, moving to target position");
                    
                    mission_state_ = MissionState::FLY_OVER_ACROSS;
                    
                    // Generate second fly-over waypoint: over target at z=7
                    {
                        const Waypoint& target_wp = waypoints_[interrupted_waypoint_index_];
                        
                        auto waypoint_msg = geometry_msgs::msg::PoseStamped();
                        waypoint_msg.header.stamp = this->now();
                        waypoint_msg.header.frame_id = "map";
                        waypoint_msg.pose.position.x = target_wp.x;
                        waypoint_msg.pose.position.y = target_wp.y;
                        waypoint_msg.pose.position.z = 7.0;
                        waypoint_msg.pose.orientation.w = 1.0;
                        
                        waypoint_pub_->publish(waypoint_msg);
                        
                        RCLCPP_INFO(this->get_logger(), 
                            "FLY-OVER Step 2: Moving to (%.2f, %.2f, 7.00)", 
                            target_wp.x, target_wp.y);
                    }
                    break;
                    
                case MissionState::FLY_OVER_ACROSS:
                    RCLCPP_INFO(this->get_logger(), 
                        "FLY-OVER complete, resuming mission");
                    
                    // Resume normal mission at interrupted waypoint
                    mission_state_ = MissionState::NORMAL;
                    current_waypoint_index_ = interrupted_waypoint_index_;
                    publish_current_waypoint();
                    break;
            }
        }
    }
    
    void publish_current_waypoint() {
        if (current_waypoint_index_ >= waypoints_.size()) {
            return;
        }
        
        const Waypoint& wp = waypoints_[current_waypoint_index_];
        
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        
        msg.pose.position.x = wp.x;
        msg.pose.position.y = wp.y;
        msg.pose.position.z = wp.z;
        msg.pose.orientation.w = 1.0;
        
        waypoint_pub_->publish(msg);
        
        RCLCPP_INFO(this->get_logger(), 
            "Published mission waypoint %zu/%zu: (%.2f, %.2f, %.2f)", 
            current_waypoint_index_ + 1, waypoints_.size(), 
            wp.x, wp.y, wp.z);
    }
    
    // Member variables
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_status_pub_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr waypoint_reached_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_detected_sub_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::vector<Waypoint> waypoints_;
    size_t current_waypoint_index_;
    bool mission_complete_;
    bool waypoint_reached_;
    bool first_waypoint_published_;
    
    MissionState mission_state_;
    size_t interrupted_waypoint_index_;
    
    Waypoint current_position_;
    double position_tolerance_;
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