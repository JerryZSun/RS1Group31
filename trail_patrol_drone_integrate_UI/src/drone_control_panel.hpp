/**
 * @file drone_control_panel.hpp
 * @brief RViz2 panel for drone control and mission management
 *
 * This header defines the DroneControlPanel class which provides a Qt-based
 * graphical user interface for controlling autonomous drone missions within RViz2.
 *
 * Features:
 * - Mission control (start, pause, resume, emergency stop)
 * - Custom waypoint queue management
 * - Waypoint library for click-to-go navigation
 * - Manual control mode with directional buttons
 * - Real-time telemetry display (position, altitude, speed)
 * - Mission progress and status monitoring
 *
 * @author RS1 Group 31
 * @date 2025
 */

#ifndef DRONE_CONTROL_PANEL_HPP
#define DRONE_CONTROL_PANEL_HPP

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QListWidget>
#include <QGroupBox>
#include <QSlider>
#include <QRadioButton>
#include <QButtonGroup>
#include <QScrollArea>
#include <memory>
#include <cmath>
#include <vector>
#include <tuple>

namespace drone_ui {

/**
 * @brief RViz2 panel widget for comprehensive drone control and monitoring
 *
 * This class implements a Qt-based control panel that integrates with RViz2
 * to provide a graphical interface for drone mission management. It supports
 * multiple control modes:
 *
 * **Mission Mode:**
 * - Start/pause/resume/stop predefined missions
 * - Add custom waypoints to mission queue
 * - Clear custom waypoints while preserving mission path
 * - Monitor mission progress and waypoint completion
 *
 * **Manual Mode:**
 * - Direct velocity control with arrow buttons
 * - Altitude control with up/down commands
 * - Rotation control with left/right commands
 * - Emergency stop for immediate halt
 *
 * **Monitoring:**
 * - Real-time position display (X, Y, Z)
 * - Current altitude indication
 * - Drone speed calculation and display
 * - Mission state and progress tracking
 * - Status messages with timestamps
 *
 * The panel publishes commands to mission and navigation nodes via ROS2 topics
 * and subscribes to telemetry and status updates for real-time display.
 *
 * @note This panel requires mission.cpp and navigation.cpp nodes to be running
 * @see Mission, Navigation
 */
class DroneControlPanel : public rviz_common::Panel {
    Q_OBJECT

public:
    explicit DroneControlPanel(QWidget* parent = nullptr);
    virtual ~DroneControlPanel();
    
    virtual void load(const rviz_common::Config& config) override;
    virtual void save(rviz_common::Config config) const override;

private Q_SLOTS:
    // Mission control button handlers
    void onStartClicked(); 
    void onStopClicked();
    void onPauseClicked();
    void onResumeClicked();
    
    // Waypoint Manager button handlers (Sequential missions)
    void onAddWaypointClicked();
    void onClearWaypointsClicked();
    
    // Waypoint Library handlers (Click-to-go navigation)
    void onLoadLibraryFile();
    void onLibraryWaypointClicked(const QString& waypoint_data);
    
    // Manual control handlers
    void onStopMovementPressed();
    
    // Mode toggle handler
    void onModeChanged();
    
    // Timer callback for ROS spinning
    void updatePanel();

private:
    // UI Setup
    void createLayout();
    void setupMissionControls();
    void setupStatusDisplay();
    void setupTelemetryDisplay();
    void setupWaypointControls();
    void setupWaypointLibrary();
    void setupManualControls();
    void styleButtons();
    void styleMissionButtons();
    void styleManualButtons();
    
    // ROS Setup
    void setupROSCommunication();
    void createPublishers();
    void createSubscribers();
    
    // ROS Callbacks
    void stateCallback(const std_msgs::msg::String::SharedPtr msg);
    void progressCallback(const std_msgs::msg::String::SharedPtr msg);
    void statusMessageCallback(const std_msgs::msg::String::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    // Manual control helpers
    void publishManualVelocity(double vx, double vy, double vz, double angular_z = 0.0);
    void stopDrone();
    void startContinuousMovement(double vx, double vy, double vz);
    void stopContinuousMovement();
    double getManualSpeed();
    
    // Helper functions
    void updateStateDisplay(const QString& state);
    void setButtonsEnabled(bool start, bool stop, bool pause, bool resume); 
    void setManualControlsEnabled(bool enabled);
    QString getColorForState(const QString& state);
    double calculateSpeed(double vx, double vy, double vz);
    void storeWaypointData(QListWidgetItem* item, double x, double y, double z);
    bool getWaypointData(QListWidgetItem* item, double& x, double& y, double& z);
    
    // ========== ROS2 Components ==========
    rclcpp::Node::SharedPtr ros_node_; 
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pause_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr resume_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr waypoint_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr clear_waypoints_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr manual_cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr library_goal_pub_;  // For click-to-go navigation
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr progress_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_message_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // ========== Qt UI Components ==========
    // Mission Control Buttons
    QPushButton* start_button_;
    QPushButton* stop_button_;
    QPushButton* pause_button_;
    QPushButton* resume_button_;
    
    // Status displays
    QLabel* state_label_;
    QLabel* state_indicator_;
    QLabel* progress_label_;
    QLabel* status_message_label_;
    QLabel* timestamp_label_;
    
    // Telemetry displays
    QLabel* position_x_label_;
    QLabel* position_y_label_;
    QLabel* position_z_label_;
    QLabel* altitude_label_;
    QLabel* speed_label_;
    
    // Waypoint Manager controls (Sequential missions)
    QDoubleSpinBox* waypoint_x_input_;
    QDoubleSpinBox* waypoint_y_input_;
    QDoubleSpinBox* waypoint_z_input_;
    QPushButton* add_waypoint_button_;
    QPushButton* clear_waypoints_button_;
    QListWidget* waypoint_list_;  // Mission queue for sequential navigation
    
    // Waypoint Library controls (Click-to-go navigation)  
    QPushButton* load_library_button_;
    QListWidget* library_list_;  // Clickable library of loaded waypoints
    QLabel* library_file_label_;
    QLabel* library_target_label_;
    std::vector<std::tuple<double, double, double, QString>> library_waypoints_data_;
    
    // Manual control components
    QRadioButton* autonomous_mode_radio_;
    QRadioButton* manual_mode_radio_;
    QButtonGroup* mode_button_group_;
    QPushButton* forward_button_;
    QPushButton* backward_button_;
    QPushButton* left_button_;
    QPushButton* right_button_;
    QPushButton* up_button_;
    QPushButton* down_button_;
    QPushButton* rotate_left_button_;    
    QPushButton* rotate_right_button_;
    QSlider* speed_slider_;
    QLabel* speed_value_label_;
    QLabel* mode_status_label_;
    
    // Layouts
    QVBoxLayout* main_layout_;
    
    // Timers
    QTimer* update_timer_;
    QTimer* movement_timer_;
    QTimer* stop_timer_;
    
    // ========== State Variables ==========
    QString current_state_;
    bool is_paused_;
    bool is_manual_mode_;
    
    // Manual control state
    double current_vx_;
    double current_vy_;
    double current_vz_;
    double current_angular_z_;
    bool button_held_;
};

}  // namespace drone_ui

#endif  // DRONE_CONTROL_PANEL_HPP
