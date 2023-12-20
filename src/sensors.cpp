#include <chrono>
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <random>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "autorccar_interfaces/msg/gnss.hpp"
#include "autorccar_interfaces/msg/imu.hpp"
#include "builtin_interfaces/msg/time.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


class Sensors : public rclcpp::Node {

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;

    rclcpp::Publisher<autorccar_interfaces::msg::Imu>::SharedPtr pub_imu;
    rclcpp::Publisher<autorccar_interfaces::msg::Gnss>::SharedPtr pub_gnss;

    std::shared_ptr<rclcpp::Node> node_;
    
    struct GNSS {
        builtin_interfaces::msg::Time timestamp;
        Eigen::Vector3d pos;
        Eigen::Vector3d vel;
    } gnss_data;

public:
    Sensors() : Node("isaac_simulation") {
        sub_imu = this->create_subscription<sensor_msgs::msg::Imu>("isaac/imu", 10, std::bind(&Sensors::callback_imu, this, _1));
        sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("isaac/odom", 10, std::bind(&Sensors::callback_odom, this, _1));

        pub_imu = this->create_publisher<autorccar_interfaces::msg::Imu>("sensors/imu", 10);
        pub_gnss = this->create_publisher<autorccar_interfaces::msg::Gnss>("sensors/gnss", 10);

        // Timer for the 10Hz publisher
        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&Sensors::publish_gnss, this));
    }

    void callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
        autorccar_interfaces::msg::Imu sim_imu;

        std::random_device rd;
        std::default_random_engine generator(rd());
        std::normal_distribution<double> noise_gyro(0.0, 0.1); // 평균 0, 표준편차 1
        std::normal_distribution<double> noise_acc(0.0, 0.1); // 평균 0, 표준편차 1
        

        sim_imu.timestamp.sec = msg->header.stamp.sec;
        sim_imu.timestamp.nanosec = msg->header.stamp.nanosec;

        sim_imu.angular_velocity.x = msg->angular_velocity.x + noise_gyro(generator);
        sim_imu.angular_velocity.y = msg->angular_velocity.y + noise_gyro(generator);
        sim_imu.angular_velocity.z = msg->angular_velocity.z + noise_gyro(generator);
        
        sim_imu.linear_acceleration.x = msg->linear_acceleration.x + noise_acc(generator);
        sim_imu.linear_acceleration.y = msg->linear_acceleration.y + noise_acc(generator);
        sim_imu.linear_acceleration.z = msg->linear_acceleration.z + noise_acc(generator);

        pub_imu->publish(sim_imu);
    }

    void callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {

        Eigen::Vector3d orgllh, tmpxyz, pos_enu, difxyz, xyz;
        Eigen::Matrix3d R, inv_R;

        orgllh << 37.5665*(M_PI/180), 128.9780*(M_PI/180), 17; // latitude(rad), longitude(rad), height(m)
        tmpxyz << -304478.8, 404380.0, 386742.7;               // meter

        // Robot forward (X) = North
        pos_enu << -msg->pose.pose.position.y, msg->pose.pose.position.x, msg->pose.pose.position.z;

        double phi = orgllh(0);
        double lam = orgllh(1);
        double sinphi = sin(phi);
        double cosphi = cos(phi);
        double sinlam = sin(lam);
        double coslam = cos(lam);

        R << -sinlam,         coslam,        0,
             -sinphi*coslam, -sinphi*sinlam, cosphi,
              cosphi*coslam,  cosphi*sinlam, sinphi;
        inv_R = R.inverse();
        difxyz << inv_R * pos_enu;
        xyz = tmpxyz + pos_enu;

        // NED Frame
        gnss_data.timestamp.sec = msg->header.stamp.sec; // + msg->timestamp.nanosec * 1e-9;
        gnss_data.timestamp.nanosec = msg->header.stamp.nanosec;
        
        gnss_data.pos << xyz(0), xyz(1), xyz(2);

        gnss_data.vel << msg->twist.twist.linear.x,
                         msg->twist.twist.linear.y,
                         msg->twist.twist.linear.z;
    }

    void publish_gnss() {
        autorccar_interfaces::msg::Gnss sim_gnss; // ECEF

        std::random_device rd;
        std::default_random_engine generator(rd());
        std::normal_distribution<double> distribution(0.0, 0.1); // 평균 0, 표준편차 1

        sim_gnss.timestamp.sec = gnss_data.timestamp.sec;
        sim_gnss.timestamp.nanosec = gnss_data.timestamp.nanosec;

        sim_gnss.position_ecef.x = gnss_data.pos[0] + distribution(generator);
        sim_gnss.position_ecef.y = gnss_data.pos[1] + distribution(generator);
        sim_gnss.position_ecef.z = gnss_data.pos[2] + distribution(generator);

        sim_gnss.velocity_ecef.x = 0;
        sim_gnss.velocity_ecef.y = 0;
        sim_gnss.velocity_ecef.z = 0;
        
        pub_gnss->publish(sim_gnss);
    }    

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Sensors>());

  rclcpp::shutdown();
  return 0;
}