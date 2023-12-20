#include <chrono>
#include <iostream>
#include <stdio.h>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class Simulation : public rclcpp::Node {

private:
    void publish_cmd_vel(double duration, double linear_x, double angular_z) {
        // auto start_time = std::chrono::system_clock::now();
        auto start_time = this->get_clock()->now();
        auto end_time = start_time + rclcpp::Duration::from_seconds(duration);
        rclcpp::Rate rate(1);  // 1 Hz rate로 설정

        while (this->get_clock()->now() < end_time) {
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = linear_x;
            cmd.angular.z = angular_z;

            pub_cmd->publish(cmd);

            rate.sleep();
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd;
    int count_;



public:
    Simulation() : Node("isaac_simulation_run") {

        pub_cmd = this->create_publisher<geometry_msgs::msg::Twist>("isaac/cmd_vel", 10);

        publish_cmd_vel(5, 2, 0); // 5sec 동안 

        publish_cmd_vel(2, 2, 8); // 2sec 동안 반시계방향
        
        publish_cmd_vel(5, 4, 0);

        publish_cmd_vel(2, 2, -8);

        publish_cmd_vel(5, 4, 0);

        publish_cmd_vel(2, 2, 8);

        publish_cmd_vel(5, 5, 0);

        publish_cmd_vel(2, 2, -8);

        publish_cmd_vel(5, 5, 0);

        publish_cmd_vel(2, 2, 8);

        publish_cmd_vel(5, 6, 0);

        publish_cmd_vel(10, 0, 0);

        RCLCPP_INFO(this->get_logger(), "Simulation Done...");
        rclcpp::shutdown();
        
    }


};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Simulation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}