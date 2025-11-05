#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include <vector>
#include <cmath>

struct Waypoint {
    double x, y, z;
};

class Mission : public rclcpp::Node {
public:
    Mission() : Node("mission"), 
                current_waypoint_index_(0), 
                mission_complete_(false),
                waypoint_reached_(false),
                first_waypoint_published_(false) {
        
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
        
        // Timer for mission management
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), // 2Hz mission management
            std::bind(&Mission::mission_loop, this)
        );
        
        // Initialize waypoints - climb up, survey trail, return home
        waypoints_ = {
            {0, 0, 0.1},
            {0, 0, 7},
            {9.5, -12, 7},
            {9.5, -12, 0.6},
            {7, -9.5, 0.6},
            {4, -8.5, 0.6},
            {2, -7.5, 0.6},
            {0, -1.5, 0.6},
            {3, 2.5, 0.6},
            {4, 3, 0.6},
            {6, 5, 0.6},
            {8, 7.5, 0.6},
            {11, 10.5, 0.6},
            {11, 10.5, 7},
            {0, 0, 7},
            {0, 0, 0.1},
        };
        
        position_tolerance_ = 0.5; // 50cm tolerance
        
        RCLCPP_INFO(this->get_logger(), 
            "Mission node initialized with %zu waypoints", waypoints_.size());
        
        // Don't publish first waypoint immediately - let subscribers connect first
        RCLCPP_INFO(this->get_logger(), 
            "Waiting for navigation node to connect...");
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
            RCLCPP_INFO(this->get_logger(), 
                "Navigation confirmed waypoint %zu reached", 
                current_waypoint_index_ + 1);
        }
    }
    
    void mission_loop() {
        // Publish first waypoint once we have subscribers
        if (!first_waypoint_published_) {
            if (waypoint_pub_->get_subscription_count() > 0) {
                RCLCPP_INFO(this->get_logger(), 
                    "Navigation node connected! Publishing first waypoint...");
                publish_current_waypoint();
                first_waypoint_published_ = true;
            } else {
                // Still waiting for subscribers
                static int wait_counter = 0;
                if (++wait_counter % 4 == 0) { // Log every 2 seconds
                    RCLCPP_INFO(this->get_logger(), 
                        "Still waiting for navigation node... (%d subscribers)", 
                        waypoint_pub_->get_subscription_count());
                }
            }
            return; // Don't process mission until first waypoint is sent
        }
        
        // Check if mission complete
        if (current_waypoint_index_ >= waypoints_.size()) {
            if (!mission_complete_) {
                RCLCPP_INFO(this->get_logger(), 
                    "=== MISSION COMPLETE: All %zu waypoints reached ===", 
                    waypoints_.size());
                mission_complete_ = true;
                
                // Publish mission complete status
                auto status_msg = std_msgs::msg::Bool();
                status_msg.data = true;
                mission_status_pub_->publish(status_msg);
            }
            return;
        }
        
        // Check if waypoint was reached
        if (waypoint_reached_) {
            waypoint_reached_ = false;
            current_waypoint_index_++;
            
            // Check if we have more waypoints
            if (current_waypoint_index_ < waypoints_.size()) {
                RCLCPP_INFO(this->get_logger(), 
                    "Moving to waypoint %zu of %zu", 
                    current_waypoint_index_ + 1, waypoints_.size());
                publish_current_waypoint();
            }
        }
        
        // Periodically republish current waypoint to ensure navigation has it
        // This helps if navigation node restarts or misses a message
        // static int republish_counter = 0;
        // if (++republish_counter % 20 == 0) { // Every 10 seconds
        //     if (waypoint_pub_->get_subscription_count() > 0) {
        //         RCLCPP_DEBUG(this->get_logger(), 
        //             "Re-publishing current waypoint %zu (periodic refresh)", 
        //             current_waypoint_index_ + 1);
        //         publish_current_waypoint();
        //     }
        // }
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
        
        // Orientation doesn't matter for waypoint navigation
        msg.pose.orientation.w = 1.0;
        
        waypoint_pub_->publish(msg);
        
        RCLCPP_INFO(this->get_logger(), 
            "Published waypoint %zu: (%.2f, %.2f, %.2f)", 
            current_waypoint_index_ + 1, wp.x, wp.y, wp.z);
    }
    
    // Member variables
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_status_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr waypoint_reached_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::vector<Waypoint> waypoints_;
    size_t current_waypoint_index_;
    bool mission_complete_;
    bool waypoint_reached_;
    bool first_waypoint_published_;
    
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