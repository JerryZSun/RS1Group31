#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("dummy_drone_controller");
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    int repeats = 100;
    double turn_l_r = 0.0;
    double move_l_r = 0.0;
    double move_u_d = 0.0;
    double move_f_b = 0.1;

    if(argc == 6){
        repeats = std::atoi(argv[1]);
        turn_l_r = std::atof(argv[2]);
        move_l_r = std::atof(argv[3]);
        move_u_d = std::atof(argv[4]);
        move_f_b = std::atof(argv[5]);
    }

    rclcpp::WallRate loop_rate(100);

    for(int i = 0; i < repeats; ++i){
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = move_f_b;
        cmd.linear.y = move_l_r;
        cmd.linear.z = move_u_d;
        cmd.angular.z = turn_l_r;

        publisher->publish(cmd);

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    geometry_msgs::msg::Twist stop_cmd;
    publisher->publish(stop_cmd);

    rclcpp::shutdown();
    return 0;
}
