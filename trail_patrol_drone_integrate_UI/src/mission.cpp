#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include <vector>
#include <cmath>
#include <string>
#include <chrono>
#include <algorithm>

struct Waypoint {
    double x, y, z;
};

enum class MissionState {
    NORMAL,
    FLY_AROUND_RIGHT,
    FLY_AROUND_LEFT,
    FLY_AROUND_RETURN,
    FLY_OVER_UP,
    FLY_OVER_ACROSS
};

class Mission : public rclcpp::Node {
public:
    Mission()
        : Node("mission"),
          current_waypoint_index_(0),
          mission_complete_(false),
          waypoint_reached_(false),
          first_waypoint_published_(false),
          mission_started_(false),
          is_paused_(false),
          mission_state_(MissionState::NORMAL),
          interrupted_waypoint_index_(0),
          avoidance_direction_(0),
          waiting_for_avoidance_direction_(false) {

        waypoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mission/current_waypoint", 10);
        mission_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/mission/complete", 10);
        ui_state_pub_ = this->create_publisher<std_msgs::msg::String>("/drone/state", 10);
        ui_progress_pub_ = this->create_publisher<std_msgs::msg::String>("/drone/mission_progress", 10);
        ui_status_pub_ = this->create_publisher<std_msgs::msg::String>("/drone/status_message", 10);
        avoidance_mode_pub_ = this->create_publisher<std_msgs::msg::Bool>("/mission/avoidance_mode", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,
            std::bind(&Mission::odom_callback, this, std::placeholders::_1));

        waypoint_reached_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/navigation/waypoint_reached", 10,
            std::bind(&Mission::waypoint_reached_callback, this, std::placeholders::_1));

        obstacle_detected_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/navigation/obstacle_detected", 10,
            std::bind(&Mission::obstacle_detected_callback, this, std::placeholders::_1));

        avoidance_direction_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/navigation/avoidance_direction", 10,
            std::bind(&Mission::avoidance_direction_callback, this, std::placeholders::_1));

        start_mission_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/drone/start_mission", 10,
            std::bind(&Mission::start_mission_callback, this, std::placeholders::_1));

        pause_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/drone/pause", 10,
            std::bind(&Mission::pause_callback, this, std::placeholders::_1));

        resume_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/drone/resume", 10,
            std::bind(&Mission::resume_callback, this, std::placeholders::_1));

        emergency_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/drone/emergency_stop", 10,
            std::bind(&Mission::emergency_stop_callback, this, std::placeholders::_1));

        add_waypoint_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/drone/add_waypoint", 10,
            std::bind(&Mission::add_waypoint_callback, this, std::placeholders::_1));

        clear_waypoints_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/drone/clear_waypoints", 10,
            std::bind(&Mission::clear_waypoints_callback, this, std::placeholders::_1));

        library_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&Mission::library_goal_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&Mission::mission_loop, this));

        waypoints_ = {
            {7.75, -8.96, 6.0}, {6.31, -8.95, 6.0}, {5.38, -7.15, 6.0},
            {3.07, -5.32, 6.0}, {2.01, -1.64, 6.0}, {1.28, 0.88, 6.0},
            {2.48, 2.17, 6.0}, {4.63, 4.89, 6.0}, {6.08, 6.08, 6.0},
            {7.30, 7.19, 6.0}, {8.44, 8.63, 6.0}, {10.37, 10.09, 6.0},
            {11.0, 10.5, 6.0}};

        position_tolerance_ = 0.5;

        RCLCPP_INFO(this->get_logger(),
            "Mission node initialized with %zu waypoints", waypoints_.size());
        publish_ui_state("IDLE");
        publish_ui_status("Mission ready with " + std::to_string(waypoints_.size()) +
                          " waypoints. Press START to begin.");
        publish_ui_progress("0/" + std::to_string(waypoints_.size()) + " waypoints");
    }

private:
    // === Callbacks ===
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_position_.x = msg->pose.pose.position.x;
        current_position_.y = msg->pose.pose.position.y;
        current_position_.z = msg->pose.pose.position.z;

        auto &q = msg->pose.pose.orientation;
        current_yaw_ = std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    void waypoint_reached_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            waypoint_reached_ = true;
        }
    }

    void obstacle_detected_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!mission_started_ || is_paused_) {
            return;
        }

        if (msg->data && mission_state_ == MissionState::NORMAL) {
            interrupted_waypoint_index_ = current_waypoint_index_;
            waiting_for_avoidance_direction_ = true;
            publish_ui_status("Obstacle detected! Planning avoidance path...");
            RCLCPP_WARN(this->get_logger(),
                "Obstacle detected near waypoint %zu - awaiting avoidance direction",
                current_waypoint_index_ + 1);
        }
    }

    void avoidance_direction_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        if (!mission_started_ || is_paused_ || !waiting_for_avoidance_direction_) {
            return;
        }

        avoidance_direction_ = msg->data;
        waiting_for_avoidance_direction_ = false;

        if (avoidance_direction_ == 0) {
            activate_fly_over_protocol();
        } else if (avoidance_direction_ == 1) {
            activate_fly_around_protocol(true);
        } else if (avoidance_direction_ == 2) {
            activate_fly_around_protocol(false);
        } else {
            RCLCPP_WARN(this->get_logger(),
                "Unknown avoidance direction (%d). Defaulting to fly-over.", avoidance_direction_);
            activate_fly_over_protocol();
        }
    }

    void start_mission_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data) {
            return;
        }
        if (waypoints_.empty()) {
            publish_ui_status("Cannot start mission: no waypoints.");
            publish_ui_state("IDLE");
            publish_ui_progress("0/0 waypoints");
            return;
        }

        mission_started_ = true;
        mission_complete_ = false;
        is_paused_ = false;
        current_waypoint_index_ = 0;
        first_waypoint_published_ = false;
        mission_state_ = MissionState::NORMAL;
        waiting_for_avoidance_direction_ = false;
        avoidance_direction_ = 0;

        publish_ui_state("NAVIGATING");
        publish_ui_status("Mission started! Publishing waypoint 1...");
        publish_ui_progress("1/" + std::to_string(waypoints_.size()));
    }

    void pause_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && mission_started_ && !is_paused_) {
            is_paused_ = true;
            publish_ui_state("PAUSED");
            publish_ui_status("Mission paused at waypoint " +
                              std::to_string(current_waypoint_index_ + 1));
        }
    }

    void resume_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && mission_started_ && is_paused_) {
            is_paused_ = false;
            publish_ui_state("NAVIGATING");
            publish_ui_status("Mission resumed.");
        }
    }

    void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            mission_started_ = false;
            is_paused_ = false;
            mission_complete_ = true;
            publish_ui_state("EMERGENCY_LANDING");
            publish_ui_status("Emergency stop! Drone descending.");
        }
    }

    void add_waypoint_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        Waypoint wp{msg->x, msg->y, msg->z > 0.0 ? msg->z : 1.5};
        waypoints_.push_back(wp);
        publish_ui_status("Waypoint added (" + std::to_string(static_cast<int>(wp.x)) + ", " +
                          std::to_string(static_cast<int>(wp.y)) + ", " +
                          std::to_string(static_cast<int>(wp.z)) + ")");
        if (!mission_started_) {
            publish_ui_progress("0/" + std::to_string(waypoints_.size()) + " waypoints");
        }
    }

    void clear_waypoints_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data) {
            return;
        }
        size_t cleared = waypoints_.size();
        waypoints_.clear();
        mission_started_ = false;
        mission_complete_ = false;
        publish_ui_state("IDLE");
        publish_ui_status("Waypoints cleared (" + std::to_string(cleared) + ")");
        publish_ui_progress("0/0 waypoints");
    }

    void library_goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr goal) {
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        msg.pose = goal->pose;
        waypoint_pub_->publish(msg);

        publish_ui_status("Library waypoint dispatched: (" +
                          std::to_string(goal->pose.position.x) + ", " +
                          std::to_string(goal->pose.position.y) + ", " +
                          std::to_string(goal->pose.position.z) + ")");
    }

    // === Mission Control ===
    void mission_loop() {
        if (!mission_started_ || waypoints_.empty()) {
            return;
        }
        if (is_paused_) {
            return;
        }

        if (!first_waypoint_published_) {
            if (waypoint_pub_->get_subscription_count() > 0) {
                publish_current_waypoint();
                first_waypoint_published_ = true;
            }
            return;
        }

        if (mission_state_ == MissionState::NORMAL &&
            current_waypoint_index_ >= waypoints_.size()) {
            if (!mission_complete_) {
                mission_complete_ = true;
                mission_started_ = false;

                auto status_msg = std_msgs::msg::Bool();
                status_msg.data = true;
                mission_status_pub_->publish(status_msg);

                publish_ui_state("MISSION_COMPLETE");
                publish_ui_status("Mission complete! All waypoints reached.");
                publish_ui_progress("Complete: " +
                    std::to_string(waypoints_.size()) + "/" +
                    std::to_string(waypoints_.size()));
            }
            return;
        }

        if (!waypoint_reached_) {
            return;
        }
        waypoint_reached_ = false;

        switch (mission_state_) {
            case MissionState::NORMAL:
                current_waypoint_index_++;
                if (current_waypoint_index_ < waypoints_.size()) {
                    publish_current_waypoint();
                }
                break;

            case MissionState::FLY_AROUND_RIGHT:
            case MissionState::FLY_AROUND_LEFT:
                mission_state_ = MissionState::FLY_AROUND_RETURN;
                publish_current_waypoint(interrupted_waypoint_index_);
                publish_ui_status("Returning to mission route after sidestep");
                break;

            case MissionState::FLY_AROUND_RETURN: {
                set_avoidance_mode(false);
                mission_state_ = MissionState::NORMAL;
                current_waypoint_index_ = interrupted_waypoint_index_;
                publish_ui_status("Obstacle bypassed. Resuming mission.");
                publish_ui_state("NAVIGATING");
                publish_current_waypoint();
                break;
            }

            case MissionState::FLY_OVER_UP: {
                mission_state_ = MissionState::FLY_OVER_ACROSS;
                const Waypoint &wp = waypoints_[interrupted_waypoint_index_];
                publish_ui_status("Fly-over: traversing obstacle from above");
                publish_waypoint(wp.x, wp.y, 7.0);
                break;
            }

            case MissionState::FLY_OVER_ACROSS:
                mission_state_ = MissionState::NORMAL;
                current_waypoint_index_ = interrupted_waypoint_index_;
                publish_ui_status("Fly-over complete. Resuming mission.");
                publish_ui_state("NAVIGATING");
                publish_current_waypoint();
                break;
        }
    }

    void activate_fly_around_protocol(bool go_right) {
        mission_state_ = go_right ? MissionState::FLY_AROUND_RIGHT : MissionState::FLY_AROUND_LEFT;
        set_avoidance_mode(true);

        double perpendicular_angle = go_right ? (current_yaw_ - M_PI / 2.0)
                                              : (current_yaw_ + M_PI / 2.0);
        double offset = 2.0;
        double target_x = current_position_.x + offset * std::cos(perpendicular_angle);
        double target_y = current_position_.y + offset * std::sin(perpendicular_angle);
        double target_z = current_position_.z;

        publish_ui_status(std::string("Executing fly-around ") + (go_right ? "right" : "left"));
        publish_ui_state("AVOIDING_OBSTACLE");
        publish_waypoint(target_x, target_y, target_z);
    }

    void activate_fly_over_protocol() {
        mission_state_ = MissionState::FLY_OVER_UP;
        set_avoidance_mode(false);
        publish_ui_status("Executing fly-over: climbing above obstacle");
        publish_ui_state("AVOIDING_OBSTACLE");
        publish_waypoint(current_position_.x, current_position_.y, 7.0);
    }

    void set_avoidance_mode(bool active) {
        auto msg = std_msgs::msg::Bool();
        msg.data = active;
        avoidance_mode_pub_->publish(msg);
    }

    std::string make_progress_string(size_t index) const {
        size_t clamped = std::min(index + 1, waypoints_.size());
        return std::to_string(clamped) + "/" + std::to_string(waypoints_.size());
    }

    void publish_current_waypoint(size_t index_override, bool update_progress = true) {
        current_waypoint_index_ = index_override;
        publish_current_waypoint(update_progress);
    }

    void publish_current_waypoint(bool update_progress = true) {
        if (current_waypoint_index_ >= waypoints_.size()) {
            return;
        }
        const Waypoint &wp = waypoints_[current_waypoint_index_];
        publish_waypoint(wp.x, wp.y, wp.z);

        std::string progress = make_progress_string(current_waypoint_index_);
        publish_ui_status("Navigating to waypoint " + progress + "...");
        if (update_progress) {
            publish_ui_progress(progress);
        }
    }

    void publish_waypoint(double x, double y, double z) {
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        msg.pose.position.x = x;
        msg.pose.position.y = y;
        msg.pose.position.z = z;
        msg.pose.orientation.w = 1.0;
        waypoint_pub_->publish(msg);
    }

    void publish_ui_state(const std::string &state) {
        auto msg = std_msgs::msg::String();
        msg.data = state;
        ui_state_pub_->publish(msg);
    }

    void publish_ui_progress(const std::string &progress) {
        auto msg = std_msgs::msg::String();
        msg.data = progress;
        ui_progress_pub_->publish(msg);
    }

    void publish_ui_status(const std::string &status) {
        auto msg = std_msgs::msg::String();
        msg.data = status;
        ui_status_pub_->publish(msg);
    }

    // === Member variables ===
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ui_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ui_progress_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ui_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr avoidance_mode_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr waypoint_reached_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_detected_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr avoidance_direction_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_mission_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr resume_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr add_waypoint_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr clear_waypoints_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr library_goal_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<Waypoint> waypoints_;
    size_t current_waypoint_index_;
    bool mission_complete_;
    bool waypoint_reached_;
    bool first_waypoint_published_;
    bool mission_started_;
    bool is_paused_;

    MissionState mission_state_;
    size_t interrupted_waypoint_index_;
    int avoidance_direction_;
    bool waiting_for_avoidance_direction_;

    Waypoint current_position_;
    double current_yaw_;
    double position_tolerance_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Mission>();

    try {
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
