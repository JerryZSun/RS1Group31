#include "drone_control_panel.hpp"
#include <QGridLayout>
#include <QFrame>
#include <QFont>
#include <QGroupBox>
#include <QScrollArea>
#include <QFileDialog>
#include <QMessageBox>
#include <QTextStream>
#include <QVariant>
#include <QVariantMap>
#include <fstream>
#include <sstream>
#include <vector>
#include <tuple>

namespace drone_ui {

DroneControlPanel::DroneControlPanel(QWidget* parent)
    : rviz_common::Panel(parent),
      current_state_("IDLE"),
      is_paused_(false),
      is_manual_mode_(false),
      current_vx_(0.0),
      current_vy_(0.0),
      current_vz_(0.0),
      current_angular_z_(0.0),
      button_held_(false) {
    
    // Create ROS2 node for this panel
    ros_node_ = std::make_shared<rclcpp::Node>("drone_control_panel");
    
    // Setup ROS communication
    setupROSCommunication();
    
    // Create timers for manual control
    movement_timer_ = new QTimer(this);
    movement_timer_->setInterval(50);  // 20Hz update rate
    connect(movement_timer_, &QTimer::timeout, [this]() {
    publishManualVelocity(current_vx_, current_vy_, current_vz_, current_angular_z_);
});
    
    stop_timer_ = new QTimer(this);
    stop_timer_->setSingleShot(true);
    connect(stop_timer_, &QTimer::timeout, this, &DroneControlPanel::stopContinuousMovement);
    
    // Create UI layout
    createLayout();
    
    // Style the buttons
    styleButtons();
    
    // Create timer for ROS spinning (100Hz)
    update_timer_ = new QTimer(this);
    connect(update_timer_, &QTimer::timeout, this, &DroneControlPanel::updatePanel);
    update_timer_->start(10);  // 10ms = 100Hz
    
    // Initialize display
    updateStateDisplay("IDLE");
    setButtonsEnabled(true, false, false, false);
    setManualControlsEnabled(false);
}

DroneControlPanel::~DroneControlPanel() {
    update_timer_->stop();
    movement_timer_->stop();
    stop_timer_->stop();
}

// This is UI Setup

void DroneControlPanel::createLayout() {
    // Create main container widget using Qwidget
    QWidget* container = new QWidget();
    main_layout_ = new QVBoxLayout();
    main_layout_->setSpacing(10);
    main_layout_->setContentsMargins(10, 10, 10, 10);
    
    // Title
    QLabel* title = new QLabel("<b>Drone Mission Control</b>");
    title->setAlignment(Qt::AlignCenter);
    QFont title_font = title->font();
    title_font.setPointSize(12);
    title->setFont(title_font);
    main_layout_->addWidget(title);
    
    // Add separator line
    QFrame* line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
    main_layout_->addWidget(line);
    
    // Status display section
    setupStatusDisplay();
    
    // Add separator line
    QFrame* line2 = new QFrame();
    line2->setFrameShape(QFrame::HLine);
    line2->setFrameShadow(QFrame::Sunken);
    main_layout_->addWidget(line2);
    
    // Telemetry display section
    setupTelemetryDisplay();
    
    // Add separator line
    QFrame* line3 = new QFrame();
    line3->setFrameShape(QFrame::HLine);
    line3->setFrameShadow(QFrame::Sunken);
    main_layout_->addWidget(line3);
    
    // Manual control section
    setupManualControls();
    
    // Add separator line
    QFrame* line4 = new QFrame();
    line4->setFrameShape(QFrame::HLine);
    line4->setFrameShadow(QFrame::Sunken);
    main_layout_->addWidget(line4);
    
    // Mission control buttons
    setupMissionControls();
    
    // Add separator line
    QFrame* line5 = new QFrame();
    line5->setFrameShape(QFrame::HLine);
    line5->setFrameShadow(QFrame::Sunken);
    main_layout_->addWidget(line5);
    
    // Waypoint controls section
    setupWaypointControls();
    
    // Add separator line
    QFrame* line6 = new QFrame();
    line6->setFrameShape(QFrame::HLine);
    line6->setFrameShadow(QFrame::Sunken);
    main_layout_->addWidget(line6);
    
    // Waypoint Library section (for click to choose the waypoint.txt file)
    setupWaypointLibrary();
    
    // Add stretch to push everything to the top
    main_layout_->addStretch();
    
    // Set layout to container
    container->setLayout(main_layout_);
    
    // Create scroll area
    QScrollArea* scroll_area = new QScrollArea();
    scroll_area->setWidget(container);
    scroll_area->setWidgetResizable(true);
    scroll_area->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll_area->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    
    // Set scroll area as the main layout
    QVBoxLayout* outer_layout = new QVBoxLayout();
    outer_layout->setContentsMargins(0, 0, 0, 0);
    outer_layout->addWidget(scroll_area);
    
    setLayout(outer_layout);
}

void DroneControlPanel::setupStatusDisplay() {
    // State display with color indicator
    QHBoxLayout* state_layout = new QHBoxLayout();
    
    state_indicator_ = new QLabel("●");  // Unicode circle
    QFont indicator_font = state_indicator_->font();
    indicator_font.setPointSize(16);
    state_indicator_->setFont(indicator_font);
    state_indicator_->setStyleSheet("QLabel { color: gray; }");
    
    state_label_ = new QLabel("State: IDLE");
    QFont state_font = state_label_->font();
    state_font.setPointSize(11);
    state_font.setBold(true);
    state_label_->setFont(state_font);
    
    state_layout->addWidget(state_indicator_);
    state_layout->addWidget(state_label_);
    state_layout->addStretch();
    
    main_layout_->addLayout(state_layout);
    
    // Progress display
    progress_label_ = new QLabel("Progress: No mission");
    progress_label_->setStyleSheet("QLabel { color: #555555; }");
    main_layout_->addWidget(progress_label_);
    
    // Status message display
    QLabel* status_title = new QLabel("Status:");
    status_title->setStyleSheet("QLabel { color: #777777; font-size: 9pt; }");
    main_layout_->addWidget(status_title);
    
    status_message_label_ = new QLabel("Waiting for commands...");
    status_message_label_->setWordWrap(true);
    status_message_label_->setStyleSheet(
        "QLabel { "
        "  background-color: #f0f0f0; "
        "  padding: 8px; "
        "  border-radius: 4px; "
        "  color: #333333; "
        "}"
    );
    status_message_label_->setMinimumHeight(60);
    status_message_label_->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    main_layout_->addWidget(status_message_label_);
    
    // Timestamp
    timestamp_label_ = new QLabel("");
    timestamp_label_->setStyleSheet("QLabel { color: #999999; font-size: 8pt; }");
    timestamp_label_->setAlignment(Qt::AlignRight);
    main_layout_->addWidget(timestamp_label_);
}

void DroneControlPanel::setupTelemetryDisplay() {
    // Create a group box for telemetry
    QGroupBox* telemetry_group = new QGroupBox("Live Telemetry");
    QVBoxLayout* telemetry_layout = new QVBoxLayout();
    
    // Position display
    QLabel* position_title = new QLabel("<b>Position:</b>");
    telemetry_layout->addWidget(position_title);
    
    QGridLayout* position_grid = new QGridLayout();
    
    position_x_label_ = new QLabel("X: -- m");
    position_y_label_ = new QLabel("Y: -- m");
    position_z_label_ = new QLabel("Z: -- m");
    
    position_x_label_->setStyleSheet("QLabel { font-family: monospace; color: #2196F3; font-weight: bold; }");
    position_y_label_->setStyleSheet("QLabel { font-family: monospace; color: #4CAF50; font-weight: bold; }");
    position_z_label_->setStyleSheet("QLabel { font-family: monospace; color: #FF9800; font-weight: bold; }");
    
    position_grid->addWidget(position_x_label_, 0, 0);
    position_grid->addWidget(position_y_label_, 0, 1);
    position_grid->addWidget(position_z_label_, 0, 2);
    
    telemetry_layout->addLayout(position_grid);
    
    // Altitude display 
    altitude_label_ = new QLabel("Altitude: -- m");
    altitude_label_->setStyleSheet(
        "QLabel { "
        "  font-size: 11pt; "
        "  font-weight: bold; "
        "  color: #FF5722; "
        "  padding: 5px; "
        "  background-color: #FFF3E0; "
        "  border-radius: 4px; "
        "}"
    );
    telemetry_layout->addWidget(altitude_label_);
    
    // Speed display
    speed_label_ = new QLabel("Speed: -- m/s");
    speed_label_->setStyleSheet(
        "QLabel { "
        "  font-size: 10pt; "
        "  color: #9C27B0; "
        "  padding: 3px; "
        "  font-weight: bold; "
        "}"
    );
    telemetry_layout->addWidget(speed_label_);
    
    telemetry_group->setLayout(telemetry_layout);
    main_layout_->addWidget(telemetry_group);
}

void DroneControlPanel::setupManualControls() {
    QGroupBox* manual_group = new QGroupBox("Manual Flight Control");
    QVBoxLayout* manual_layout = new QVBoxLayout();
    
    // Mode selection
    QHBoxLayout* mode_layout = new QHBoxLayout();
    QLabel* mode_label = new QLabel("Control Mode:");
    mode_label->setStyleSheet("QLabel { font-weight: bold; }");
    
    autonomous_mode_radio_ = new QRadioButton("Autonomous");
    manual_mode_radio_ = new QRadioButton("Manual");
    
    mode_button_group_ = new QButtonGroup(this);
    mode_button_group_->addButton(autonomous_mode_radio_, 0);
    mode_button_group_->addButton(manual_mode_radio_, 1);
    
    autonomous_mode_radio_->setChecked(true);  // Default to autonomous
    
    connect(autonomous_mode_radio_, &QRadioButton::toggled, this, &DroneControlPanel::onModeChanged);
    connect(manual_mode_radio_, &QRadioButton::toggled, this, &DroneControlPanel::onModeChanged);
    
    mode_layout->addWidget(mode_label);
    mode_layout->addWidget(autonomous_mode_radio_);
    mode_layout->addWidget(manual_mode_radio_);
    mode_layout->addStretch();
    manual_layout->addLayout(mode_layout);
    
    // Mode status label
    mode_status_label_ = new QLabel("Mode: Autonomous Navigation");
    mode_status_label_->setStyleSheet(
        "QLabel { "
        "  background-color: #E3F2FD; "
        "  color: #1976D2; "
        "  padding: 5px; "
        "  border-radius: 4px; "
        "  font-weight: bold; "
        "}"
    );
    manual_layout->addWidget(mode_status_label_);
    
    // Speed control
    QHBoxLayout* speed_layout = new QHBoxLayout();
    QLabel* speed_label = new QLabel("Speed:");
    speed_label->setStyleSheet("QLabel { font-weight: bold; }");
    
    speed_slider_ = new QSlider(Qt::Horizontal);
    speed_slider_->setRange(10, 100);  // 10% to 100%
    speed_slider_->setValue(50);  // Default 50%
    speed_slider_->setTickPosition(QSlider::TicksBelow);
    speed_slider_->setTickInterval(10);
    
    speed_value_label_ = new QLabel("50%");
    speed_value_label_->setMinimumWidth(40);
    speed_value_label_->setStyleSheet("QLabel { font-weight: bold; color: #FF5722; }");
    
    connect(speed_slider_, &QSlider::valueChanged, [this](int value) {
        speed_value_label_->setText(QString("%1%").arg(value));
    });
    
    speed_layout->addWidget(speed_label);
    speed_layout->addWidget(speed_slider_);
    speed_layout->addWidget(speed_value_label_);
    manual_layout->addLayout(speed_layout);
    
    // Directional control buttons
    QLabel* controls_label = new QLabel("Movement Controls:");
    controls_label->setStyleSheet("QLabel { font-weight: bold; margin-top: 5px; }");
    manual_layout->addWidget(controls_label);
    
    // Horizontal movement grid
    QGridLayout* horizontal_grid = new QGridLayout();
    
    forward_button_ = new QPushButton("⬆ Forward");
    backward_button_ = new QPushButton("⬇ Backward");
    left_button_ = new QPushButton("⬅ Left");
    right_button_ = new QPushButton("➡ Right");
    
    // Forward button - tap vs hold
    connect(forward_button_, &QPushButton::pressed, this, [this]() {
        button_held_ = false;
        stop_timer_->stop();
        double speed = getManualSpeed();
        startContinuousMovement(speed, 0, 0);
        status_message_label_->setText("Moving FORWARD");
        timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
        
        QTimer::singleShot(300, this, [this]() {
            if (forward_button_->isDown()) {
                button_held_ = true;
            }
        });
    });
    connect(forward_button_, &QPushButton::released, this, [this]() {
        stopContinuousMovement();
        if (!button_held_) {
            double speed = getManualSpeed();
            publishManualVelocity(speed, 0, 0);
            stop_timer_->start(500);
        }
        status_message_label_->setText("Forward movement stopped");
        timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
    });
    
    // Backward button - tap vs hold
    connect(backward_button_, &QPushButton::pressed, this, [this]() {
        button_held_ = false;
        stop_timer_->stop();
        double speed = getManualSpeed();
        startContinuousMovement(-speed, 0, 0);
        status_message_label_->setText("Moving BACKWARD");
        timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
        
        QTimer::singleShot(300, this, [this]() {
            if (backward_button_->isDown()) {
                button_held_ = true;
            }
        });
    });
    connect(backward_button_, &QPushButton::released, this, [this]() {
        stopContinuousMovement();
        if (!button_held_) {
            double speed = getManualSpeed();
            publishManualVelocity(-speed, 0, 0);
            stop_timer_->start(500);
        }
        status_message_label_->setText("Backward movement stopped");
        timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
    });
    
    // Left button - tap and hold
    connect(left_button_, &QPushButton::pressed, this, [this]() {
        button_held_ = false;
        stop_timer_->stop();
        double speed = getManualSpeed();
        startContinuousMovement(0, speed, 0);
        status_message_label_->setText("Moving LEFT");
        timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
        
        QTimer::singleShot(300, this, [this]() {
            if (left_button_->isDown()) {
                button_held_ = true;
            }
        });
    });
    connect(left_button_, &QPushButton::released, this, [this]() {
        stopContinuousMovement();
        if (!button_held_) {
            double speed = getManualSpeed();
            publishManualVelocity(0, speed, 0);
            stop_timer_->start(500);
        }
        status_message_label_->setText("Left movement stopped");
        timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
    });
    
    // Right button - tap and hold
    connect(right_button_, &QPushButton::pressed, this, [this]() {
        button_held_ = false;
        stop_timer_->stop();
        double speed = getManualSpeed();
        startContinuousMovement(0, -speed, 0);
        status_message_label_->setText("Moving RIGHT");
        timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
        
        QTimer::singleShot(300, this, [this]() {
            if (right_button_->isDown()) {
                button_held_ = true;
            }
        });
    });
    connect(right_button_, &QPushButton::released, this, [this]() {
        stopContinuousMovement();
        if (!button_held_) {
            double speed = getManualSpeed();
            publishManualVelocity(0, -speed, 0);
            stop_timer_->start(500);
        }
        status_message_label_->setText("Right movement stopped");
        timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
    });
    
    horizontal_grid->addWidget(forward_button_, 0, 1);
    horizontal_grid->addWidget(left_button_, 1, 0);
    horizontal_grid->addWidget(right_button_, 1, 2);
    horizontal_grid->addWidget(backward_button_, 2, 1);
    
    manual_layout->addLayout(horizontal_grid);
    
    // Vertical movement
    QHBoxLayout* vertical_layout = new QHBoxLayout();
    
    up_button_ = new QPushButton("⬆ Climb");
    down_button_ = new QPushButton("⬇ Descend");


// Rotation controls
QLabel* rotation_label = new QLabel("Rotation Controls:");
rotation_label->setStyleSheet("QLabel { font-weight: bold; margin-top: 5px; }");
manual_layout->addWidget(rotation_label);

QHBoxLayout* rotation_layout = new QHBoxLayout();

rotate_left_button_ = new QPushButton("⟲ Rotate Left");
rotate_right_button_ = new QPushButton("⟳ Rotate Right");

// Rotate Left button - tap and hold
connect(rotate_left_button_, &QPushButton::pressed, this, [this]() {
    button_held_ = false;
    stop_timer_->stop();
    double angular_speed = getManualSpeed() * 1.0;  // 1.0 rad/s max
    startContinuousMovement(0, 0, 0);
    current_angular_z_ = angular_speed;
    publishManualVelocity(0, 0, 0, angular_speed);
    status_message_label_->setText("Rotating LEFT");
    timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
    
    QTimer::singleShot(300, this, [this]() {
        if (rotate_left_button_->isDown()) {
            button_held_ = true;
        }
    });
});
connect(rotate_left_button_, &QPushButton::released, this, [this]() {
    if (button_held_) {
        stopContinuousMovement();
    } else {
        // Rotate Left Button
        double angular_speed = getManualSpeed() * 1.0;
        publishManualVelocity(0, 0, 0, angular_speed);
        stop_timer_->start(500);
    }
    current_angular_z_ = 0.0;
    status_message_label_->setText("Rotation stopped");
    timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
});

// Rotate Right button 
connect(rotate_right_button_, &QPushButton::pressed, this, [this]() {
    button_held_ = false;
    stop_timer_->stop();
    double angular_speed = getManualSpeed() * 1.0;  // 1.0 rad/s max
    startContinuousMovement(0, 0, 0);
    current_angular_z_ = -angular_speed;
    publishManualVelocity(0, 0, 0, -angular_speed);
    status_message_label_->setText("Rotating RIGHT");
    timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
    
    QTimer::singleShot(300, this, [this]() {
        if (rotate_right_button_->isDown()) {
            button_held_ = true;
        }
    });
});
connect(rotate_right_button_, &QPushButton::released, this, [this]() {
    if (button_held_) {
        stopContinuousMovement();
    } else {
        
        double angular_speed = getManualSpeed() * 1.0;
        publishManualVelocity(0, 0, 0, -angular_speed);
        stop_timer_->start(500);
    }
    current_angular_z_ = 0.0;
    status_message_label_->setText("Rotation stopped");
    timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
});

rotation_layout->addWidget(rotate_left_button_);
rotation_layout->addWidget(rotate_right_button_);
manual_layout->addLayout(rotation_layout);
    
    // Up button 
    connect(up_button_, &QPushButton::pressed, this, [this]() {
        button_held_ = false;
        stop_timer_->stop();
        double speed = getManualSpeed();
        startContinuousMovement(0, 0, speed * 0.7);
        status_message_label_->setText("CLIMBING");
        timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
        
        QTimer::singleShot(300, this, [this]() {
            if (up_button_->isDown()) {
                button_held_ = true;
            }
        });
    });
    connect(up_button_, &QPushButton::released, this, [this]() {
        stopContinuousMovement();
        if (!button_held_) {
            double speed = getManualSpeed();
            publishManualVelocity(0, 0, speed * 0.7);
            stop_timer_->start(500);
        }
        status_message_label_->setText("Climb stopped");
        timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
    });
    
    // Down button 
    connect(down_button_, &QPushButton::pressed, this, [this]() {
        button_held_ = false;
        stop_timer_->stop();
        double speed = getManualSpeed();
        startContinuousMovement(0, 0, -speed * 0.7);
        status_message_label_->setText("DESCENDING");
        timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
        
        QTimer::singleShot(300, this, [this]() {
            if (down_button_->isDown()) {
                button_held_ = true;
            }
        });
    });
    connect(down_button_, &QPushButton::released, this, [this]() {
        stopContinuousMovement();
        if (!button_held_) {
            double speed = getManualSpeed();
            publishManualVelocity(0, 0, -speed * 0.7);
            stop_timer_->start(500);
        }
        status_message_label_->setText("Descend stopped");
        timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
    });
    
    vertical_layout->addWidget(up_button_);
    vertical_layout->addWidget(down_button_);
    manual_layout->addLayout(vertical_layout);
    
    // Stop button
    
    manual_group->setLayout(manual_layout);
    main_layout_->addWidget(manual_group);
}

void DroneControlPanel::setupWaypointControls() {
    QGroupBox* waypoint_group = new QGroupBox("Waypoint Manager");
    QVBoxLayout* waypoint_layout = new QVBoxLayout();
    
    // ========== MANUAL WAYPOINT INPUT ==========
    QLabel* manual_title = new QLabel("<b>Manual Waypoint Entry:</b>");
    manual_title->setStyleSheet("QLabel { color: #1976D2; margin-top: 5px; }");
    waypoint_layout->addWidget(manual_title);
    
    QLabel* instructions = new QLabel("Add waypoints (meters):");
    instructions->setStyleSheet("QLabel { color: #666666; font-size: 9pt; }");
    waypoint_layout->addWidget(instructions);
    
    QGridLayout* input_grid = new QGridLayout();
    
    QLabel* x_label = new QLabel("X:");
    x_label->setStyleSheet("QLabel { font-weight: bold; }");
    waypoint_x_input_ = new QDoubleSpinBox();
    waypoint_x_input_->setRange(-100.0, 100.0);
    waypoint_x_input_->setValue(10.0);
    waypoint_x_input_->setDecimals(1);
    waypoint_x_input_->setSingleStep(0.5);
    waypoint_x_input_->setStyleSheet("QDoubleSpinBox { padding: 5px; }");
    
    QLabel* y_label = new QLabel("Y:");
    y_label->setStyleSheet("QLabel { font-weight: bold; }");
    waypoint_y_input_ = new QDoubleSpinBox();
    waypoint_y_input_->setRange(-100.0, 100.0);
    waypoint_y_input_->setValue(-10.0);
    waypoint_y_input_->setDecimals(1);
    waypoint_y_input_->setSingleStep(0.5);
    waypoint_y_input_->setStyleSheet("QDoubleSpinBox { padding: 5px; }");
    
    QLabel* z_label = new QLabel("Z:");
    z_label->setStyleSheet("QLabel { font-weight: bold; }");
    waypoint_z_input_ = new QDoubleSpinBox();
    waypoint_z_input_->setRange(0.5, 10.0);
    waypoint_z_input_->setValue(0);
    waypoint_z_input_->setDecimals(1);
    waypoint_z_input_->setSingleStep(0.5);
    waypoint_z_input_->setStyleSheet("QDoubleSpinBox { padding: 5px; }");
    
    input_grid->addWidget(x_label, 0, 0);
    input_grid->addWidget(waypoint_x_input_, 0, 1);
    input_grid->addWidget(y_label, 0, 2);
    input_grid->addWidget(waypoint_y_input_, 0, 3);
    input_grid->addWidget(z_label, 0, 4);
    input_grid->addWidget(waypoint_z_input_, 0, 5);
    
    waypoint_layout->addLayout(input_grid);
    
    QHBoxLayout* manual_button_layout = new QHBoxLayout();
    
    add_waypoint_button_ = new QPushButton("➕ Add to Queue");
    add_waypoint_button_->setStyleSheet(
        "QPushButton {"
        "  background-color: #03A9F4;"
        "  color: white;"
        "  border: none;"
        "  border-radius: 4px;"
        "  padding: 8px;"
        "  font-weight: bold;"
        "}"
        "QPushButton:hover {"
        "  background-color: #0288D1;"
        "}"
        "QPushButton:pressed {"
        "  background-color: #01579B;"
        "}"
    );
    connect(add_waypoint_button_, &QPushButton::clicked, 
            this, &DroneControlPanel::onAddWaypointClicked);
    
    manual_button_layout->addWidget(add_waypoint_button_);
    waypoint_layout->addLayout(manual_button_layout);
    
    // Separator
    QFrame* separator1 = new QFrame();
    separator1->setFrameShape(QFrame::HLine);
    separator1->setFrameShadow(QFrame::Sunken);
    separator1->setStyleSheet("QFrame { color: #CCCCCC; }");
    waypoint_layout->addWidget(separator1);
    
    // ========== MISSION QUEUE SECTION ==========
    QLabel* queue_title = new QLabel("<b>Mission Waypoint Queue:</b>");
    queue_title->setStyleSheet("QLabel { color: #D32F2F; margin-top: 5px; }");
    waypoint_layout->addWidget(queue_title);
    
    QLabel* list_label = new QLabel("Active waypoints for mission:");
    list_label->setStyleSheet("QLabel { color: #666666; font-size: 9pt; }");
    waypoint_layout->addWidget(list_label);
    
    waypoint_list_ = new QListWidget();
    waypoint_list_->setMaximumHeight(120);
    waypoint_list_->setStyleSheet(
        "QListWidget {"
        "  border: 2px solid #F44336;"
        "  border-radius: 4px;"
        "  padding: 5px;"
        "  background-color: #FFEBEE;"
        "}"
        "QListWidget::item {"
        "  color: #C62828;"
        "  font-weight: bold;"
        "}"
    );
    waypoint_layout->addWidget(waypoint_list_);
    
    clear_waypoints_button_ = new QPushButton("🗑️ Clear Queue");
    clear_waypoints_button_->setStyleSheet(
        "QPushButton {"
        "  background-color: #757575;"
        "  color: white;"
        "  border: none;"
        "  border-radius: 4px;"
        "  padding: 8px;"
        "}"
        "QPushButton:hover {"
        "  background-color: #616161;"
        "}"
        "QPushButton:pressed {"
        "  background-color: #424242;"
        "}"
    );
    connect(clear_waypoints_button_, &QPushButton::clicked, 
            this, &DroneControlPanel::onClearWaypointsClicked);
    waypoint_layout->addWidget(clear_waypoints_button_);
    
    waypoint_group->setLayout(waypoint_layout);
    main_layout_->addWidget(waypoint_group);
}

// ========== WAYPOINT LIBRARY SETUP  ==========

void DroneControlPanel::setupWaypointLibrary() {
    QGroupBox* library_group = new QGroupBox("Waypoint Library (Click-to-Go Navigation)");
    QVBoxLayout* library_layout = new QVBoxLayout();
    
    QLabel* library_desc = new QLabel("Load waypoint file and click any waypoint to navigate directly:");
    library_desc->setStyleSheet("QLabel { color: #666666; font-size: 9pt; font-style: italic; }");
    library_layout->addWidget(library_desc);
    
    // Load File Button
    load_library_button_ = new QPushButton("📂 Load Waypoint File");
    load_library_button_->setStyleSheet(
        "QPushButton {"
        "  background-color: #4CAF50;"
        "  color: white;"
        "  border: none;"
        "  border-radius: 4px;"
        "  padding: 10px;"
        "  font-weight: bold;"
        "  font-size: 10pt;"
        "}"
        "QPushButton:hover {"
        "  background-color: #45a049;"
        "}"
        "QPushButton:pressed {"
        "  background-color: #2E7D32;"
        "}"
    );
    connect(load_library_button_, &QPushButton::clicked, 
            this, &DroneControlPanel::onLoadLibraryFile);
    library_layout->addWidget(load_library_button_);
    
    // File label
    library_file_label_ = new QLabel("No file loaded");
    library_file_label_->setStyleSheet(
        "QLabel { "
        "  color: #999999; "
        "  font-size: 9pt; "
        "  font-style: italic; "
        "  padding: 3px; "
        "}"
    );
    library_layout->addWidget(library_file_label_);
    
    // Library list label
    QLabel* list_label = new QLabel("Available Waypoints (click to navigate):");
    list_label->setStyleSheet("QLabel { color: #1976D2; font-weight: bold; margin-top: 5px; }");
    library_layout->addWidget(list_label);
    
    // Clickable waypoint library
    library_list_ = new QListWidget();
    library_list_->setMaximumHeight(150);
    library_list_->setStyleSheet(
        "QListWidget {"
        "  border: 2px solid #4CAF50;"
        "  border-radius: 4px;"
        "  padding: 5px;"
        "  background-color: #F1F8E9;"
        "}"
        "QListWidget::item {"
        "  padding: 8px;"
        "  margin: 2px;"
        "  background-color: white;"
        "  border: 1px solid #C8E6C9;"
        "  border-radius: 3px;"
        "}"
        "QListWidget::item:hover {"
        "  background-color: #C8E6C9;"
        "  cursor: pointer;"
        "}"
        "QListWidget::item:selected {"
        "  background-color: #66BB6A;"
        "  color: white;"
        "  font-weight: bold;"
        "}"
    );
    
    connect(library_list_, &QListWidget::itemClicked, 
            [this](QListWidgetItem* item) {
                onLibraryWaypointClicked(item->data(Qt::UserRole).toString());
            });
    
    library_layout->addWidget(library_list_);
    
    // Target label
    library_target_label_ = new QLabel("Current Target: None");
    library_target_label_->setStyleSheet(
        "QLabel { "
        "  background-color: #E3F2FD; "
        "  color: #1976D2; "
        "  padding: 5px; "
        "  border-radius: 4px; "
        "  font-weight: bold; "
        "}"
    );
    library_layout->addWidget(library_target_label_);
    
    // Hint
    QLabel* hint = new QLabel("💡 Tip: These waypoints navigate directly, not as a mission queue");
    hint->setStyleSheet("QLabel { color: #666666; font-size: 8pt; font-style: italic; margin-top: 3px; }");
    library_layout->addWidget(hint);
    
    library_group->setLayout(library_layout);
    main_layout_->addWidget(library_group);
}

void DroneControlPanel::setupMissionControls() {
    start_button_ = new QPushButton("START MISSION");
    start_button_->setMinimumHeight(45);
    connect(start_button_, &QPushButton::clicked, this, &DroneControlPanel::onStartClicked);
    main_layout_->addWidget(start_button_);
    
    stop_button_ = new QPushButton("EMERGENCY STOP");
    stop_button_->setMinimumHeight(45);
    connect(stop_button_, &QPushButton::clicked, this, &DroneControlPanel::onStopClicked);
    main_layout_->addWidget(stop_button_);
    
    QHBoxLayout* pause_resume_layout = new QHBoxLayout();
    
    pause_button_ = new QPushButton("PAUSE");
    pause_button_->setMinimumHeight(40);
    connect(pause_button_, &QPushButton::clicked, this, &DroneControlPanel::onPauseClicked);
    
    resume_button_ = new QPushButton("RESUME");
    resume_button_->setMinimumHeight(40);
    connect(resume_button_, &QPushButton::clicked, this, &DroneControlPanel::onResumeClicked);
    
    pause_resume_layout->addWidget(pause_button_);
    pause_resume_layout->addWidget(resume_button_);
    
    main_layout_->addLayout(pause_resume_layout);
}

void DroneControlPanel::styleButtons() {
    styleMissionButtons();
    styleManualButtons();
}

void DroneControlPanel::styleMissionButtons() {
    start_button_->setStyleSheet(
        "QPushButton {"
        "  background-color: #4CAF50;"
        "  color: white;"
        "  border: none;"
        "  border-radius: 6px;"
        "  font-size: 12pt;"
        "  font-weight: bold;"
        "}"
        "QPushButton:hover {"
        "  background-color: #45a049;"
        "}"
        "QPushButton:pressed {"
        "  background-color: #3d8b40;"
        "}"
        "QPushButton:disabled {"
        "  background-color: #cccccc;"
        "  color: #666666;"
        "}"
    );
    
    stop_button_->setStyleSheet(
        "QPushButton {"
        "  background-color: #f44336;"
        "  color: white;"
        "  border: none;"
        "  border-radius: 6px;"
        "  font-size: 12pt;"
        "  font-weight: bold;"
        "}"
        "QPushButton:hover {"
        "  background-color: #da190b;"
        "}"
        "QPushButton:pressed {"
        "  background-color: #c41508;"
        "}"
        "QPushButton:disabled {"
        "  background-color: #cccccc;"
        "  color: #666666;"
        "}"
    );
    
    pause_button_->setStyleSheet(
        "QPushButton {"
        "  background-color: #ff9800;"
        "  color: white;"
        "  border: none;"
        "  border-radius: 6px;"
        "  font-size: 11pt;"
        "  font-weight: bold;"
        "}"
        "QPushButton:hover {"
        "  background-color: #e68900;"
        "}"
        "QPushButton:pressed {"
        "  background-color: #cc7a00;"
        "}"
        "QPushButton:disabled {"
        "  background-color: #cccccc;"
        "  color: #666666;"
        "}"
    );
    
    resume_button_->setStyleSheet(
        "QPushButton {"
        "  background-color: #2196F3;"
        "  color: white;"
        "  border: none;"
        "  border-radius: 6px;"
        "  font-size: 11pt;"
        "  font-weight: bold;"
        "}"
        "QPushButton:hover {"
        "  background-color: #0b7dda;"
        "}"
        "QPushButton:pressed {"
        "  background-color: #0a6ebd;"
        "}"
        "QPushButton:disabled {"
        "  background-color: #cccccc;"
        "  color: #666666;"
        "}"
    );
}

void DroneControlPanel::styleManualButtons() {
    QString direction_button_style = 
        "QPushButton {"
        "  background-color: #2196F3;"
        "  color: white;"
        "  border: none;"
        "  border-radius: 4px;"
        "  padding: 10px;"
        "  font-weight: bold;"
        "  min-height: 40px;"
        "}"
        "QPushButton:hover {"
        "  background-color: #1976D2;"
        "}"
        "QPushButton:pressed {"
        "  background-color: #0D47A1;"
        "}"
        "QPushButton:disabled {"
        "  background-color: #BDBDBD;"
        "  color: #757575;"
        "}";
    
    forward_button_->setStyleSheet(direction_button_style);
    backward_button_->setStyleSheet(direction_button_style);
    left_button_->setStyleSheet(direction_button_style);
    right_button_->setStyleSheet(direction_button_style);
    up_button_->setStyleSheet(direction_button_style);
    down_button_->setStyleSheet(direction_button_style);

    QString rotation_button_style = 
    "QPushButton {"
    "  background-color: #9C27B0;"  // Purple color
    "  color: white;"
    "  border: none;"
    "  border-radius: 4px;"
    "  padding: 10px;"
    "  font-weight: bold;"
    "  min-height: 40px;"
    "}"
    "QPushButton:hover {"
    "  background-color: #7B1FA2;"
    "}"
    "QPushButton:pressed {"
    "  background-color: #4A148C;"
    "}"
    "QPushButton:disabled {"
    "  background-color: #BDBDBD;"
    "  color: #757575;"
    "}";

rotate_left_button_->setStyleSheet(rotation_button_style);
rotate_right_button_->setStyleSheet(rotation_button_style);
}

// ========== ROS SETUP ==========

void DroneControlPanel::setupROSCommunication() {
    createPublishers();
    createSubscribers();
}

void DroneControlPanel::createPublishers() {
    start_pub_ = ros_node_->create_publisher<std_msgs::msg::Bool>("/drone/start_mission", 10);
    stop_pub_ = ros_node_->create_publisher<std_msgs::msg::Bool>("/drone/emergency_stop", 10);
    pause_pub_ = ros_node_->create_publisher<std_msgs::msg::Bool>("/drone/pause", 10);
    resume_pub_ = ros_node_->create_publisher<std_msgs::msg::Bool>("/drone/resume", 10);
    
    waypoint_pub_ = ros_node_->create_publisher<geometry_msgs::msg::Point>("/drone/add_waypoint", 10);
    clear_waypoints_pub_ = ros_node_->create_publisher<std_msgs::msg::Bool>("/drone/clear_waypoints", 10);
    
    manual_cmd_vel_pub_ = ros_node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // Library waypoint publisher 
    library_goal_pub_ = ros_node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
}

void DroneControlPanel::createSubscribers() {
    state_sub_ = ros_node_->create_subscription<std_msgs::msg::String>(
        "/drone/state", 10,
        std::bind(&DroneControlPanel::stateCallback, this, std::placeholders::_1));
    
    progress_sub_ = ros_node_->create_subscription<std_msgs::msg::String>(
        "/drone/mission_progress", 10,
        std::bind(&DroneControlPanel::progressCallback, this, std::placeholders::_1));
    
    status_message_sub_ = ros_node_->create_subscription<std_msgs::msg::String>(
        "/drone/status_message", 10,
        std::bind(&DroneControlPanel::statusMessageCallback, this, std::placeholders::_1));
    
    odom_sub_ = ros_node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry", 10,
        std::bind(&DroneControlPanel::odomCallback, this, std::placeholders::_1));
}

// ========== MISSION CONTROL BUTTON HANDLERS ==========

void DroneControlPanel::onStartClicked() {
    if (is_manual_mode_) {
        status_message_label_->setText("Cannot start mission in Manual mode! Switch to Autonomous first.");
        timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
        return;
    }
    
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    start_pub_->publish(msg);
    
    status_message_label_->setText("START command sent...");
    timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
    
    setButtonsEnabled(false, true, false, false);
}

void DroneControlPanel::onStopClicked() {
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    stop_pub_->publish(msg);
    
    if (is_manual_mode_) {
        stopContinuousMovement();
    }
    
    status_message_label_->setText("EMERGENCY STOP activated!");
    timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
    
    setButtonsEnabled(true, false, false, false);
    is_paused_ = false;
}

void DroneControlPanel::onPauseClicked() {
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    pause_pub_->publish(msg);
    
    status_message_label_->setText("Mission PAUSED");
    timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
    
    setButtonsEnabled(false, true, false, true);
    is_paused_ = true;
}

void DroneControlPanel::onResumeClicked() {
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    resume_pub_->publish(msg);
    
    status_message_label_->setText("Mission RESUMED");
    timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
    
    setButtonsEnabled(false, true, true, false);
    is_paused_ = false;
}

// ========== WAYPOINT BUTTON HANDLERS ==========

void DroneControlPanel::onAddWaypointClicked() {
    double x = waypoint_x_input_->value();
    double y = waypoint_y_input_->value();
    double z = waypoint_z_input_->value();
    
    auto waypoint_msg = geometry_msgs::msg::Point();
    waypoint_msg.x = x;
    waypoint_msg.y = y;
    waypoint_msg.z = z;
    waypoint_pub_->publish(waypoint_msg);
    
    QString waypoint_text = QString("Waypoint: X=%1, Y=%2, Z=%3")
                                .arg(x, 0, 'f', 1)
                                .arg(y, 0, 'f', 1)
                                .arg(z, 0, 'f', 1);
    waypoint_list_->addItem(waypoint_text);
    
    status_message_label_->setText(QString("Waypoint added: (%1, %2, %3)")
                                      .arg(x, 0, 'f', 1)
                                      .arg(y, 0, 'f', 1)
                                      .arg(z, 0, 'f', 1));
    timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
    
    waypoint_y_input_->setValue(y + 5.0);
}

void DroneControlPanel::onClearWaypointsClicked() {
    waypoint_list_->clear();
    
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    clear_waypoints_pub_->publish(msg);
    
    status_message_label_->setText("Mission queue cleared");
    timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
}


void DroneControlPanel::onLoadLibraryFile() {
    // Open file dialog to select waypoint file
    QString fileName = QFileDialog::getOpenFileName(
        this,
        "Load Waypoint Library File",
        QDir::homePath(),
        "Text Files (*.txt *.csv);;All Files (*)"
    );
    
    if (fileName.isEmpty()) {
        return;  // User cancelled
    }
    
    // Open and read the file
    std::ifstream file(fileName.toStdString());
    if (!file.is_open()) {
        QMessageBox::warning(this, "Error", "Failed to open file: " + fileName);
        status_message_label_->setText("ERROR: Could not open file");
        timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
        return;
    }
    
    int waypoint_count = 0;
    int line_number = 0;
    std::string line;
    
    // Clear library
    library_list_->clear();
    library_waypoints_data_.clear();
    
    // Read file line by line
    while (std::getline(file, line)) {
        line_number++;
        
        // Skip empty lines and comments (lines starting with #)
        if (line.empty() || line[0] == '#') {
            continue;
        }
        
        // Parse the line (expecting format: x, y, z or x y z)
        std::stringstream ss(line);
        double x, y, z;
        char comma;
        
        // Try comma-separated format
        if (ss >> x >> comma >> y >> comma >> z) {
            
        }
        // Try space-separated format
        else {
            ss.clear();
            ss.str(line);
            if (!(ss >> x >> y >> z)) {
                
                continue;
            }
        }
        
        // Validate waypoint values
        if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
            continue;
        }
        
        // Set default altitude if z is 0 or negative
        if (z <= 0) {
            z = 2.0;  // Default cruise altitude
        }
        
        // Create waypoint display text
        QString waypoint_text = QString("📍 WP%1: (%2, %3, %4)")
                                    .arg(waypoint_count + 1)
                                    .arg(x, 0, 'f', 1)
                                    .arg(y, 0, 'f', 1)
                                    .arg(z, 0, 'f', 1);
        
        // Store waypoint data
        QString waypoint_data = QString("%1,%2,%3").arg(x).arg(y).arg(z);
        library_waypoints_data_.push_back(std::make_tuple(x, y, z, waypoint_text));
        
        // Add to library list
        QListWidgetItem* item = new QListWidgetItem(waypoint_text);
        item->setData(Qt::UserRole, waypoint_data);
        library_list_->addItem(item);
        
        waypoint_count++;
    }
    
    file.close();
    
    // Update status
    if (waypoint_count > 0) {
        QFileInfo fileInfo(fileName);
        library_file_label_->setText(QString("Loaded: %1 (%2 waypoints)")
                                         .arg(fileInfo.fileName())
                                         .arg(waypoint_count));
        library_file_label_->setStyleSheet(
            "QLabel { color: #4CAF50; font-weight: bold; font-size: 9pt; padding: 3px; }"
        );
        
        status_message_label_->setText(
            QString("Library loaded: %1 waypoints ready (click to navigate)").arg(waypoint_count));
        timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
        
        // Log success
        RCLCPP_INFO(ros_node_->get_logger(), 
            "Loaded %d waypoints into library from: %s", 
            waypoint_count, fileName.toStdString().c_str());
    } else {
        QMessageBox::warning(this, "No Waypoints", 
            "No valid waypoints found in the file.\n\n"
            "Expected format:\n"
            "x, y, z (one waypoint per line)\n"
            "Example:\n"
            "10.0, -10.0, 2.0\n"
            "15.5, -15.5, 2.5\n"
            "20.0, -20.0, 3.0");
        status_message_label_->setText("ERROR: No valid waypoints in file");
        timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
        
        library_file_label_->setText("No file loaded");
        library_file_label_->setStyleSheet(
            "QLabel { color: #999999; font-size: 9pt; font-style: italic; padding: 3px; }"
        );
    }
}

void DroneControlPanel::onLibraryWaypointClicked(const QString& waypoint_data) {
    // Parse waypoint data
    QStringList coords = waypoint_data.split(",");
    if (coords.size() != 3) {
        RCLCPP_ERROR(ros_node_->get_logger(), "Invalid waypoint data format");
        return;
    }
    
    double x = coords[0].toDouble();
    double y = coords[1].toDouble();
    double z = coords[2].toDouble();
    
    // Create and publish goal pose
    auto goal_msg = geometry_msgs::msg::PoseStamped();
    goal_msg.header.stamp = ros_node_->now();
    goal_msg.header.frame_id = "map";
    
    goal_msg.pose.position.x = x;
    goal_msg.pose.position.y = y;
    goal_msg.pose.position.z = z;
    
    // Set orientation (facing forward)
    goal_msg.pose.orientation.w = 1.0;
    goal_msg.pose.orientation.x = 0.0;
    goal_msg.pose.orientation.y = 0.0;
    goal_msg.pose.orientation.z = 0.0;
    
    library_goal_pub_->publish(goal_msg);
    
    // Update UI
    library_target_label_->setText(QString("Current Target: (%1, %2, %3)")
                                      .arg(x, 0, 'f', 1)
                                      .arg(y, 0, 'f', 1)
                                      .arg(z, 0, 'f', 1));
    library_target_label_->setStyleSheet(
        "QLabel { "
        "  background-color: #4CAF50; "
        "  color: white; "
        "  padding: 5px; "
        "  border-radius: 4px; "
        "  font-weight: bold; "
        "}"
    );
    
    status_message_label_->setText(QString("Navigating to waypoint: (%1, %2, %3)")
                                      .arg(x, 0, 'f', 1)
                                      .arg(y, 0, 'f', 1)
                                      .arg(z, 0, 'f', 1));
    timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
    
    RCLCPP_INFO(ros_node_->get_logger(), 
        "Library waypoint clicked - Navigating to: (%.2f, %.2f, %.2f)", x, y, z);
}

// ========== MANUAL CONTROL HANDLERS ==========

void DroneControlPanel::onModeChanged() {
    is_manual_mode_ = manual_mode_radio_->isChecked();
    
    if (is_manual_mode_) {
        mode_status_label_->setText("Mode: Manual Flight Control");
        mode_status_label_->setStyleSheet(
            "QLabel { "
            "  background-color: #FFF3E0; "
            "  color: #F57C00; "
            "  padding: 5px; "
            "  border-radius: 4px; "
            "  font-weight: bold; "
            "}"
        );
        setManualControlsEnabled(true);
        setButtonsEnabled(false, true, false, false);
        
        status_message_label_->setText("Switched to MANUAL mode. Use directional buttons to control the drone.");
        timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
        
    } else {
        mode_status_label_->setText("Mode: Autonomous Navigation");
        mode_status_label_->setStyleSheet(
            "QLabel { "
            "  background-color: #E3F2FD; "
            "  color: #1976D2; "
            "  padding: 5px; "
            "  border-radius: 4px; "
            "  font-weight: bold; "
            "}"
        );
        setManualControlsEnabled(false);
        setButtonsEnabled(true, false, false, false);
        stopContinuousMovement();
        
        status_message_label_->setText("Switched to AUTONOMOUS mode. Use mission controls.");
        timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
    }
}

// ========== ROS CALLBACKS ==========

void DroneControlPanel::stateCallback(const std_msgs::msg::String::SharedPtr msg) {
    QString state = QString::fromStdString(msg->data);
    current_state_ = state;
    updateStateDisplay(state);
    
    if (!is_manual_mode_) {
        if (state == "IDLE" || state == "MISSION_COMPLETE" || state == "STOPPED") {
            setButtonsEnabled(true, false, false, false);
            is_paused_ = false;
        } else if (state == "NAVIGATING" || state == "AVOIDING_OBSTACLE") {
            if (is_paused_) {
                setButtonsEnabled(false, true, false, true);
            } else {
                setButtonsEnabled(false, true, true, false);
            }
        } else if (state == "PAUSED") {
            setButtonsEnabled(false, true, false, true);
            is_paused_ = true;
        }
    }
}

void DroneControlPanel::progressCallback(const std_msgs::msg::String::SharedPtr msg) {
    QString progress = QString::fromStdString(msg->data);
    progress_label_->setText("Progress: " + progress);
}

void DroneControlPanel::statusMessageCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (!is_manual_mode_) {
        QString message = QString::fromStdString(msg->data);
        status_message_label_->setText(message);
        timestamp_label_->setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
    }
}

void DroneControlPanel::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;
    
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    double vz = msg->twist.twist.linear.z;
    
    double speed = calculateSpeed(vx, vy, vz);
    
    position_x_label_->setText(QString("X: %1 m").arg(x, 0, 'f', 2));
    position_y_label_->setText(QString("Y: %1 m").arg(y, 0, 'f', 2));
    position_z_label_->setText(QString("Z: %1 m").arg(z, 0, 'f', 2));
    altitude_label_->setText(QString("Altitude: %1 m").arg(z, 0, 'f', 2));
    speed_label_->setText(QString("Speed: %1 m/s").arg(speed, 0, 'f', 2));
}

// ========== HELPER FUNCTIONS ==========

void DroneControlPanel::publishManualVelocity(double vx, double vy, double vz, double angular_z) {
    auto cmd = geometry_msgs::msg::Twist();
    cmd.linear.x = vx;
    cmd.linear.y = vy;
    cmd.linear.z = vz;
    cmd.angular.z = angular_z;
    manual_cmd_vel_pub_->publish(cmd);

    if (std::abs(vz) > 0.01) {
        // to use Ignition transport to set base_link velocity
        RCLCPP_WARN(ros_node_->get_logger(), 
            "Z velocity requested but VelocityControl plugin doesn't support it!");
    }
}

void DroneControlPanel::stopDrone() {
    publishManualVelocity(0, 0, 0, 0);
}

void DroneControlPanel::startContinuousMovement(double vx, double vy, double vz) {
    current_vx_ = vx;
    current_vy_ = vy;
    current_vz_ = vz;
    movement_timer_->start();
    publishManualVelocity(vx, vy, vz, current_angular_z_);
}

void DroneControlPanel::stopContinuousMovement() {
    movement_timer_->stop();
    current_vx_ = 0.0;
    current_vy_ = 0.0;
    current_vz_ = 0.0;
    stopDrone();
}

double DroneControlPanel::getManualSpeed() {
    return speed_slider_->value() / 100.0 * 1.0;
}

void DroneControlPanel::updateStateDisplay(const QString& state) {
    state_label_->setText("State: " + state);
    
    QString color = getColorForState(state);
    state_indicator_->setStyleSheet("QLabel { color: " + color + "; }");
}

QString DroneControlPanel::getColorForState(const QString& state) {
    if (state == "IDLE") {
        return "#888888";
    } else if (state == "NAVIGATING") {
        return "#4CAF50";
    } else if (state == "PAUSED") {
        return "#ff9800";
    } else if (state == "STOPPED" || state == "EMERGENCY_STOP") {
        return "#f44336";
    } else if (state == "MISSION_COMPLETE") {
        return "#2196F3";
    } else if (state == "AVOIDING_OBSTACLE") {
        return "#FFC107";
    }
    return "#888888";
}

void DroneControlPanel::setButtonsEnabled(bool start, bool stop, bool pause, bool resume) {
    start_button_->setEnabled(start);
    stop_button_->setEnabled(stop);
    pause_button_->setEnabled(pause);
    resume_button_->setEnabled(resume);
}

void DroneControlPanel::setManualControlsEnabled(bool enabled) {
    forward_button_->setEnabled(enabled);
    backward_button_->setEnabled(enabled);
    left_button_->setEnabled(enabled);
    right_button_->setEnabled(enabled);
    up_button_->setEnabled(enabled);
    down_button_->setEnabled(enabled);
    speed_slider_->setEnabled(enabled);
    rotate_left_button_->setEnabled(enabled);   
    rotate_right_button_->setEnabled(enabled);
}

double DroneControlPanel::calculateSpeed(double vx, double vy, double vz) {
    return std::sqrt(vx*vx + vy*vy + vz*vz);
}

void DroneControlPanel::updatePanel() {
    rclcpp::spin_some(ros_node_);
}

// ========== WAYPOINT DATA HELPERS ==========

void DroneControlPanel::storeWaypointData(QListWidgetItem* item, double x, double y, double z) {
    // Store waypoint coordinates as user data in the list item
    QVariantMap data;
    data["x"] = x;
    data["y"] = y;
    data["z"] = z;
    item->setData(Qt::UserRole, data);
}

bool DroneControlPanel::getWaypointData(QListWidgetItem* item, double& x, double& y, double& z) {
    QVariant variant = item->data(Qt::UserRole);
    if (!variant.isValid() || !variant.canConvert<QVariantMap>()) {
        return false;
    }
    
    QVariantMap data = variant.toMap();
    if (!data.contains("x") || !data.contains("y") || !data.contains("z")) {
        return false;
    }
    
    x = data["x"].toDouble();
    y = data["y"].toDouble();
    z = data["z"].toDouble();
    return true;
}

// ========== RVIZ PANEL INTERFACE ==========

void DroneControlPanel::load(const rviz_common::Config& config) {
    rviz_common::Panel::load(config);
}

void DroneControlPanel::save(rviz_common::Config config) const {
    rviz_common::Panel::save(config);
}

}  // namespace drone_ui

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(drone_ui::DroneControlPanel, rviz_common::Panel)
