#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include <vector>
#include <cmath>
#include <chrono>
#include <fstream>
#include <iomanip>

struct Waypoint {
    double x, y, z;
};

class UINavigationBridge : public rclcpp::Node {
public:
    UINavigationBridge()
        : Node("drone_ui_navigation_bridge"),
          current_waypoint_index_(0),
          mission_active_(false),
          is_paused_(false),
          waypoint_reached_(false),
          obstacle_detected_(false),
          total_distance_traveled_(0.0),
          max_speed_(0.0),
          altitude_sum_(0.0),
          altitude_count_(0),
          has_last_position_(false),
          statistics_initialized_(false) {
        
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "UI Navigation Bridge Starting");
        RCLCPP_INFO(this->get_logger(), "Connecting your UI to friend's navigation");
        RCLCPP_INFO(this->get_logger(), "========================================");
        
        // ============ PUBLISHERS TO UI ============
        ui_state_pub_ = create_publisher<std_msgs::msg::String>("/drone/state", 10);
        ui_progress_pub_ = create_publisher<std_msgs::msg::String>("/drone/mission_progress", 10);
        ui_status_pub_ = create_publisher<std_msgs::msg::String>("/drone/status_message", 10);
        ui_statistics_pub_ = create_publisher<std_msgs::msg::String>("/drone/mission_statistics", 10);

        // Republish odometry for UI 
        ui_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odometry", 10);
        
        // ============ PUBLISHERS TO NAVIGATION SYSTEM ============
        nav_waypoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mission/current_waypoint", 10);
        
        // ============ SUBSCRIBERS FROM UI ============
        ui_start_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/drone/start_mission", 10,
            std::bind(&UINavigationBridge::ui_start_callback, this, std::placeholders::_1));
        
        ui_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/drone/emergency_stop", 10,
            std::bind(&UINavigationBridge::ui_stop_callback, this, std::placeholders::_1));
        
        ui_pause_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/drone/pause", 10,
            std::bind(&UINavigationBridge::ui_pause_callback, this, std::placeholders::_1));
        
        ui_resume_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/drone/resume", 10,
            std::bind(&UINavigationBridge::ui_resume_callback, this, std::placeholders::_1));
        
        ui_add_waypoint_sub_ = create_subscription<geometry_msgs::msg::Point>(
            "/drone/add_waypoint", 10,
            std::bind(&UINavigationBridge::ui_add_waypoint_callback, this, std::placeholders::_1));
        
        ui_clear_waypoints_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/drone/clear_waypoints", 10,
            std::bind(&UINavigationBridge::ui_clear_waypoints_callback, this, std::placeholders::_1));
        
        // Click-to-go from waypoint library
        ui_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&UINavigationBridge::ui_goal_callback, this, std::placeholders::_1));
        
        // ============ SUBSCRIBERS FROM NAVIGATION SYSTEM ============
        nav_waypoint_reached_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/navigation/waypoint_reached", 10,
            std::bind(&UINavigationBridge::nav_waypoint_reached_callback, this, std::placeholders::_1));
        
        nav_obstacle_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/navigation/obstacle_detected", 10,
            std::bind(&UINavigationBridge::nav_obstacle_callback, this, std::placeholders::_1));
        
        // Odometry passthrough
        nav_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,  // Raw odometry from simulation
            std::bind(&UINavigationBridge::odom_callback, this, std::placeholders::_1));
        
        // ============ STATE MANAGEMENT TIMER ============
        state_timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&UINavigationBridge::state_management_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Bridge initialized successfully");
        RCLCPP_INFO(this->get_logger(), "Subscribed to UI topics: /drone/*");
        RCLCPP_INFO(this->get_logger(), "Publishing to navigation: /mission/current_waypoint");
        RCLCPP_INFO(this->get_logger(), "Listening to navigation: /navigation/*");
        RCLCPP_INFO(this->get_logger(), "Waiting for UI commands...");

        // Initialize with waypoints
        initialize_default_waypoints();

        publish_ui_state("IDLE");
        publish_ui_status("Bridge ready. Press START to begin mission with " +
                         std::to_string(waypoint_queue_.size()) + " waypoints.");
        publish_ui_progress(std::to_string(waypoint_queue_.size()) + " waypoints loaded");
    }

private:
   
    // UI COMMAND CALLBACKS//
    
    void ui_start_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data) return;
        
        if (waypoint_queue_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Cannot start: No waypoints in queue!");
            publish_ui_status("ERROR: No waypoints in queue! Add waypoints first.");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "✓ Starting mission with %zu waypoints", waypoint_queue_.size());

        mission_active_ = true;
        is_paused_ = false;
        current_waypoint_index_ = 0;

        // Reset mission statistics
        reset_statistics();

        publish_ui_state("NAVIGATING");
        publish_ui_status("Mission started! Navigating to waypoint 1...");
        publish_ui_progress("1/" + std::to_string(waypoint_queue_.size()));

        publish_next_waypoint();
    }
    
    void ui_stop_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data) return;
        
        RCLCPP_WARN(this->get_logger(), "⚠️  EMERGENCY STOP activated!");
        
        mission_active_ = false;
        is_paused_ = false;
        
        publish_ui_state("STOPPED");
        publish_ui_status("EMERGENCY STOP! Mission halted.");
    }
    
    void ui_pause_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data || !mission_active_) return;
        
        RCLCPP_INFO(this->get_logger(), "⏸️  Mission paused");
        
        is_paused_ = true;
        
        publish_ui_state("PAUSED");
        publish_ui_status("Mission paused at waypoint " + std::to_string(current_waypoint_index_ + 1));
        publish_ui_progress("Paused: " + std::to_string(current_waypoint_index_ + 1) + "/" + 
                           std::to_string(waypoint_queue_.size()));
    }
    
    void ui_resume_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data || !is_paused_) return;
        
        RCLCPP_INFO(this->get_logger(), "▶️  Mission resumed");
        
        is_paused_ = false;
        
        publish_ui_state("NAVIGATING");
        publish_ui_status("Mission resumed. Continuing navigation...");
        publish_ui_progress(std::to_string(current_waypoint_index_ + 1) + "/" + 
                           std::to_string(waypoint_queue_.size()));
        
        // If waypoint was reached during pause, move to next
        if (waypoint_reached_) {
            waypoint_reached_ = false;
            advance_to_next_waypoint();
        }
    }
    
    void ui_add_waypoint_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        Waypoint wp;
        wp.x = msg->x;
        wp.y = msg->y;
        wp.z = msg->z > 0 ? msg->z : 1.5;
        
        waypoint_queue_.push_back(wp);
        
        RCLCPP_INFO(this->get_logger(), "➕ Waypoint added: (%.2f, %.2f, %.2f)", wp.x, wp.y, wp.z);
        
        publish_ui_status("Waypoint added: (" + 
                         std::to_string((int)wp.x) + ", " + 
                         std::to_string((int)wp.y) + ", " + 
                         std::to_string((int)wp.z) + ")");
        publish_ui_progress(std::to_string(waypoint_queue_.size()) + " waypoints queued");
    }
    
    void ui_clear_waypoints_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data) return;

        size_t count = waypoint_queue_.size();
        waypoint_queue_.clear();
        current_waypoint_index_ = 0;

        // Don't stop the mission when clearing queue - let it continue
        // The mission node handles waypoint management

        RCLCPP_INFO(this->get_logger(), "🗑️  Cleared %zu waypoints from queue", count);

        // Don't publish state changes - let the mission node handle that
        publish_ui_status("Queue cleared (" + std::to_string(count) + " removed)");
        publish_ui_progress(std::to_string(waypoint_queue_.size()) + " waypoints queued");
    }
    
    void ui_goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), 
                   "🎯 Click-to-go waypoint: (%.2f, %.2f, %.2f)", 
                   msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        
        // Send directly to navigation (bypasses mission queue)
        nav_waypoint_pub_->publish(*msg);
        
        publish_ui_status("Navigating to clicked waypoint...");
        
        if (!mission_active_) {
            publish_ui_state("NAVIGATING");
        }
    }
    
    
    // NAVIGATION SYSTEM CALLBACKS // 
    
    void nav_waypoint_reached_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data) return;
        
        waypoint_reached_ = true;
        
        RCLCPP_INFO(this->get_logger(), "✓ Waypoint %zu reached", current_waypoint_index_ + 1);
        
        if (mission_active_ && !is_paused_) {
            advance_to_next_waypoint();
        }
    }
    
    void nav_obstacle_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        obstacle_detected_ = msg->data;
        
        if (msg->data) {
            RCLCPP_WARN(this->get_logger(), "⚠️  Obstacle detected by navigation");
            
            if (mission_active_) {
                publish_ui_state("AVOIDING_OBSTACLE");
                publish_ui_status("Obstacle detected - navigation handling avoidance");
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "✓ Obstacle cleared");
            
            if (mission_active_ && !is_paused_) {
                publish_ui_state("NAVIGATING");
                publish_ui_status("Obstacle cleared - resuming navigation");
            }
        }
    }
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Republish raw odom so the RViz control panel sees live telemetry
        ui_odom_pub_->publish(*msg);

        // Track statistics only during active mission (including when paused to maintain continuity)
        if (statistics_initialized_ && mission_active_) {
            // Calculate distance traveled
            if (has_last_position_) {
                double dx = msg->pose.pose.position.x - last_position_.x;
                double dy = msg->pose.pose.position.y - last_position_.y;
                double dz = msg->pose.pose.position.z - last_position_.z;
                double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
                total_distance_traveled_ += distance;
            }

            // Update last position
            last_position_.x = msg->pose.pose.position.x;
            last_position_.y = msg->pose.pose.position.y;
            last_position_.z = msg->pose.pose.position.z;
            has_last_position_ = true;

            // Track altitude for average
            altitude_sum_ += msg->pose.pose.position.z;
            altitude_count_++;

            // Calculate current speed and track max
            double vx = msg->twist.twist.linear.x;
            double vy = msg->twist.twist.linear.y;
            double vz = msg->twist.twist.linear.z;
            double current_speed = std::sqrt(vx*vx + vy*vy + vz*vz);

            if (current_speed > max_speed_) {
                max_speed_ = current_speed;
            }
        }
    }


    // WAYPOINT INITIALIZATION

    void initialize_default_waypoints() {
        waypoint_queue_.clear();

        waypoint_queue_.push_back({7.75, -8.96, 6.0});
        waypoint_queue_.push_back({6.31, -8.95, 6.0});
        waypoint_queue_.push_back({5.38, -7.15, 6.0});
        waypoint_queue_.push_back({4.36, -2.42, 6.0});
        waypoint_queue_.push_back({1.06, -5.5, 6.0});
        waypoint_queue_.push_back({1.28, 0.88, 6.0});
        waypoint_queue_.push_back({2.48, 2.17, 6.0});
        waypoint_queue_.push_back({4.63, 4.89, 6.0});
        waypoint_queue_.push_back({6.08, 6.08, 6.0});
        waypoint_queue_.push_back({7.30, 7.19, 6.0});
        waypoint_queue_.push_back({8.44, 8.63, 6.0});
        waypoint_queue_.push_back({10.37, 10.09, 6.0});
        waypoint_queue_.push_back({11.0, 10.5, 6.0});

        RCLCPP_INFO(this->get_logger(),
            "Initialized %zu default waypoints at 6m altitude", waypoint_queue_.size());
    }

    
    // MISSION MANAGEMENT

    void state_management_loop() {
        if (mission_active_ && !is_paused_) {
            // Periodic progress updates
            static int counter = 0;
            if (++counter % 4 == 0) {  // Every 2 seconds
                publish_ui_progress(std::to_string(current_waypoint_index_ + 1) + "/" +
                                   std::to_string(waypoint_queue_.size()));

                // Publish statistics periodically
                publish_statistics();
            }
        }
    }
    
    void advance_to_next_waypoint() {
        current_waypoint_index_++;

        if (current_waypoint_index_ >= waypoint_queue_.size()) {
            // Mission complete
            RCLCPP_INFO(this->get_logger(), "🎉 Mission complete! All %zu waypoints reached", waypoint_queue_.size());

            mission_active_ = false;

            // Calculate final mission time
            mission_end_time_ = this->now();

            // Publish final statistics
            publish_statistics();
            save_statistics_to_file();

            publish_ui_state("MISSION_COMPLETE");
            publish_ui_status("Mission complete! All waypoints reached.");
            publish_ui_progress("Complete: " + std::to_string(waypoint_queue_.size()) + "/" +
                               std::to_string(waypoint_queue_.size()));
        } else {
            // Next waypoint
            RCLCPP_INFO(this->get_logger(), 
                       "➡️  Moving to waypoint %zu of %zu", 
                       current_waypoint_index_ + 1, waypoint_queue_.size());
            
            publish_ui_progress(std::to_string(current_waypoint_index_ + 1) + "/" + 
                               std::to_string(waypoint_queue_.size()));
            
            publish_next_waypoint();
        }
        
        waypoint_reached_ = false;
    }
    
    void publish_next_waypoint() {
        if (current_waypoint_index_ >= waypoint_queue_.size()) return;
        
        const Waypoint& wp = waypoint_queue_[current_waypoint_index_];
        
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        
        msg.pose.position.x = wp.x;
        msg.pose.position.y = wp.y;
        msg.pose.position.z = wp.z;
        
        msg.pose.orientation.w = 1.0;
        
        nav_waypoint_pub_->publish(msg);
        
        RCLCPP_INFO(this->get_logger(), 
                   "📍 Published waypoint %zu to navigation: (%.2f, %.2f, %.2f)", 
                   current_waypoint_index_ + 1, wp.x, wp.y, wp.z);
        
        publish_ui_status("Navigating to waypoint " + 
                         std::to_string(current_waypoint_index_ + 1) + " of " + 
                         std::to_string(waypoint_queue_.size()));
    }
    
    
    // UI STATUS PUBLISHERS
    
    void publish_ui_state(const std::string& state) {
        auto msg = std_msgs::msg::String();
        msg.data = state;
        ui_state_pub_->publish(msg);
    }
    
    void publish_ui_progress(const std::string& progress) {
        auto msg = std_msgs::msg::String();
        msg.data = progress;
        ui_progress_pub_->publish(msg);
    }
    
    void publish_ui_status(const std::string& status) {
        auto msg = std_msgs::msg::String();
        msg.data = status;
        ui_status_pub_->publish(msg);
    }

   
    // STATISTICS FUNCTIONS

    void reset_statistics() {
        total_distance_traveled_ = 0.0;
        max_speed_ = 0.0;
        altitude_sum_ = 0.0;
        altitude_count_ = 0;
        has_last_position_ = false;
        mission_start_time_ = this->now();
        statistics_initialized_ = true;

        RCLCPP_INFO(this->get_logger(), "📊 Statistics tracking initialized");
    }

    void publish_statistics() {
        if (!statistics_initialized_) return;

        // Calculate time elapsed
        auto current_time = this->now();
        double time_elapsed = (current_time - mission_start_time_).seconds();

        // Calculate average altitude
        double avg_altitude = (altitude_count_ > 0) ? (altitude_sum_ / altitude_count_) : 0.0;

        // Format statistics message
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << "Distance: " << total_distance_traveled_ << "m | ";
        ss << "Time: " << static_cast<int>(time_elapsed) << "s | ";
        ss << "Avg Alt: " << avg_altitude << "m | ";
        ss << "Max Speed: " << max_speed_ << "m/s";

        auto msg = std_msgs::msg::String();
        msg.data = ss.str();
        ui_statistics_pub_->publish(msg);

        RCLCPP_DEBUG(this->get_logger(), "📊 Statistics: %s", ss.str().c_str());
    }

    void save_statistics_to_file() {
        if (!statistics_initialized_) return;

        // Calculate final statistics
        double time_elapsed = (mission_end_time_ - mission_start_time_).seconds();
        double avg_altitude = (altitude_count_ > 0) ? (altitude_sum_ / altitude_count_) : 0.0;

        // Generate filename with timestamp
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream filename;
        filename << "mission_statistics_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".txt";

        std::ofstream file(filename.str());
        if (file.is_open()) {
            file << "========================================\n";
            file << "MISSION STATISTICS REPORT\n";
            file << "========================================\n";
            file << std::fixed << std::setprecision(2);
            file << "\nMission Overview:\n";
            file << "  Total Waypoints: " << waypoint_queue_.size() << "\n";
            file << "  Waypoints Reached: " << current_waypoint_index_ << "\n";
            file << "\nPerformance Metrics:\n";
            file << "  Total Distance Traveled: " << total_distance_traveled_ << " meters\n";
            file << "  Time Elapsed: " << time_elapsed << " seconds ("
                 << static_cast<int>(time_elapsed / 60) << "m "
                 << static_cast<int>(time_elapsed) % 60 << "s)\n";
            file << "  Average Altitude: " << avg_altitude << " meters\n";
            file << "  Maximum Speed: " << max_speed_ << " m/s\n";
            file << "  Average Speed: " << (time_elapsed > 0 ? total_distance_traveled_ / time_elapsed : 0.0) << " m/s\n";
            file << "\n========================================\n";
            file << "End of Report\n";
            file << "========================================\n";
            file.close();

            RCLCPP_INFO(this->get_logger(), "📊 Statistics saved to: %s", filename.str().c_str());
            publish_ui_status("Mission stats saved to " + filename.str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to save statistics file!");
        }
    }

    
    // MEMBER VARIABLES//
   
    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ui_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ui_progress_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ui_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ui_statistics_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ui_odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav_waypoint_pub_;
    
    // Subscribers from UI
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ui_start_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ui_stop_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ui_pause_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ui_resume_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ui_add_waypoint_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ui_clear_waypoints_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ui_goal_sub_;
    
    // Subscribers from Navigation
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr nav_waypoint_reached_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr nav_obstacle_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr nav_odom_sub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr state_timer_;
    
    // State
    std::vector<Waypoint> waypoint_queue_;
    size_t current_waypoint_index_;
    bool mission_active_;
    bool is_paused_;
    bool waypoint_reached_;
    bool obstacle_detected_;

    // Statistics tracking
    double total_distance_traveled_;
    double max_speed_;
    double altitude_sum_;
    int altitude_count_;
    geometry_msgs::msg::Point last_position_;
    bool has_last_position_;
    bool statistics_initialized_;
    rclcpp::Time mission_start_time_;
    rclcpp::Time mission_end_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UINavigationBridge>();
    
    RCLCPP_INFO(node->get_logger(), "Bridge node spinning...");
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
