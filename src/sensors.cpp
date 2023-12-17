#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdio.h>

#include "sensors.h"

using namespace std::chrono_literals;
using std::placeholders::_1;


class Sensors : public rclcpp::Node {

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;

    rclcpp::Publisher<autorccar_interfaces::msg::Imu>::SharedPtr pub_imu;
    rclcpp::Publisher<autorccar_interfaces::msg::Gnss>::SharedPtr pub_gnss;

    std::shared_ptr<rclcpp::Node> node_;

public:
    Sensors() : Node("isaac_simulation") {
        sub_imu = this->create_subscription<sensor_msgs::msg::Imu>("isaac/imu", 10, std::bind(&Sensors::callback_imu, this, _1));
        sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("isaac/odom", 10, std::bind(&Sensors::callback_odom, this, _1));

        pub_imu = this->create_publisher<autorccar_interfaces::msg::Imu>("sensors/imu", 10);
        pub_gnss = this->create_publisher<autorccar_interfaces::msg::Gnss>("sensors/gnss", 10);

    }
    void callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
        autorccar_interfaces::msg::Imu sim_imu;

        // sim_imu.header.stamp = msg->header.stamp;

        sim_imu.angular_velocity.x = msg->angular_velocity.x * 10;
        sim_imu.angular_velocity.y = msg->angular_velocity.y * 10;
        sim_imu.angular_velocity.z = msg->angular_velocity.z * 10;
        
        sim_imu.linear_acceleration.x = msg->linear_acceleration.x * 10;
        sim_imu.linear_acceleration.y = msg->linear_acceleration.y * 10;
        sim_imu.linear_acceleration.z = msg->linear_acceleration.z * 10;

        pub_imu->publish(sim_imu);
    }

    void callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        autorccar_interfaces::msg::Gnss sim_gnss;

        // sim_gnss.header.stamp = msg->header.stamp;

        sim_gnss.position_ecef.x = msg->pose.pose.position.x*100;
        sim_gnss.position_ecef.y = msg->pose.pose.position.y*100;
        sim_gnss.position_ecef.z = msg->pose.pose.position.z*100;

        pub_gnss->publish(sim_gnss);
    }

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Sensors>());

  rclcpp::shutdown();
  return 0;
}