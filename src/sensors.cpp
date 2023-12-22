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
    void ReadParameters() {
        double init_lat, init_lng, init_h;

        get_parameter_or<float>("IMU.gyro_noise", param.gyro_noise, param.gyro_noise);
        get_parameter_or<float>("IMU.accel_noise", param.accel_noise, param.accel_noise);
        
        get_parameter_or<int>("GNSS.update_ms", param.update_ms, param.update_ms);
        get_parameter_or<double>("GNSS.init_lat", init_lat, init_lat);
        get_parameter_or<double>("GNSS.init_lng", init_lng, init_lng);
        get_parameter_or<double>("GNSS.init_height", init_h, init_h);
        get_parameter_or<double>("GNSS.global_position_noise", param.global_position_noise, param.global_position_noise);
        get_parameter_or<double>("GNSS.global_velocity_noise", param.global_velocity_noise, param.global_velocity_noise);
        get_parameter_or<double>("GNSS.local_position_noise", param.local_position_noise, param.local_position_noise);
        get_parameter_or<double>("GNSS.local_velocity_noise", param.local_velocity_noise, param.local_velocity_noise);

        param.orgllh << init_lat*(M_PI/180), init_lng*(M_PI/180), init_h;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;

    rclcpp::Publisher<autorccar_interfaces::msg::Imu>::SharedPtr pub_imu;
    rclcpp::Publisher<autorccar_interfaces::msg::Gnss>::SharedPtr pub_gnss_global;
    rclcpp::Publisher<autorccar_interfaces::msg::Gnss>::SharedPtr pub_gnss_local;

    std::shared_ptr<rclcpp::Node> node_;
    
    struct GNSS {
        builtin_interfaces::msg::Time timestamp;
        Eigen::Vector3d pos;
        Eigen::Vector3d vel;
    };
    GNSS gnss_global;
    GNSS gnss_local;

    struct Parameters_ {
        float gyro_noise;
        float accel_noise;
        int update_ms;
        Eigen::Vector3d orgllh;
        double global_position_noise;
        double global_velocity_noise;
        double local_position_noise;
        double local_velocity_noise;
    } param;

    double gnss_period;
    Eigen::Vector3d init_xyz;


public:
    explicit Sensors(const rclcpp::NodeOptions& options) : Node("isaac_simulation", options) {
        sub_imu = this->create_subscription<sensor_msgs::msg::Imu>("isaac/imu", 10, std::bind(&Sensors::callback_imu, this, _1));
        sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("isaac/odom", 10, std::bind(&Sensors::callback_odom, this, _1));

        pub_imu = this->create_publisher<autorccar_interfaces::msg::Imu>("sensors/imu", 10);
        pub_gnss_global = this->create_publisher<autorccar_interfaces::msg::Gnss>("sensors/gnss_global", 10);
        pub_gnss_local = this->create_publisher<autorccar_interfaces::msg::Gnss>("sensors/gnss_local", 10);

        ReadParameters();

        timer_ = create_wall_timer(std::chrono::milliseconds(param.update_ms), std::bind(&Sensors::publish_gnss, this));
    }


    void callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
        autorccar_interfaces::msg::Imu sim_imu;

        std::random_device rd;
        std::default_random_engine generator(rd());
        std::normal_distribution<double> noise_gyro(0.0, param.gyro_noise); // (평균, 표준편차)
        std::normal_distribution<double> noise_acc(0.0, param.accel_noise); // (평균, 표준편차)
        
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

    Eigen::Vector3d llh2xyz(Eigen::Vector3d &llh) {

        Eigen::Vector3d xyz = Eigen::Vector3d::Zero();        
        double phi, lambda, h;
        double a, b, e;
        double sinphi, cosphi, coslam, sinlam, tan2phi;
        double tmp, tmpden, tmp2;
        double x, y, z;

        phi = llh(0);
        lambda = llh(1);
        h = llh(2);

        a = 6378137.0000;
        b = 6356752.3142;
        e = sqrt(1-pow(b/a, 2));

        sinphi = sin(phi);
        cosphi = cos(phi);
        sinlam = sin(lambda);
        coslam = cos(lambda);
        tan2phi = tan(phi)*tan(phi);
        tmp = 1 - e*e;
        tmpden = sqrt(1 + tmp*tan2phi);

        x = (a*coslam)/tmpden + h*coslam*cosphi;
        y = (a*sinlam)/tmpden + h*sinlam*cosphi;
        tmp2 = sqrt(1-e*e*sinphi*sinphi);
        z = (a*tmp*sinphi)/tmp2 + h*sinphi;
        
        xyz << x, y, z;

        return xyz;
    }

    Eigen::Vector3d enu2xyz(Eigen::Vector3d &pos_enu, Eigen::Vector3d &orgllh) {
        Eigen::Vector3d tmpxyz, difxyz, xyz;
        Eigen::Matrix3d R, inv_R;

        tmpxyz = llh2xyz(orgllh);

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

        return xyz;
    }

    void callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {

        Eigen::Vector3d pos_enu, xyz;

        // Robot forward (X) = North
        pos_enu << -msg->pose.pose.position.y, msg->pose.pose.position.x, msg->pose.pose.position.z;
        xyz = enu2xyz(pos_enu, param.orgllh);

        // Global Position (ECEF)
        gnss_global.timestamp.sec = msg->header.stamp.sec;
        gnss_global.timestamp.nanosec = msg->header.stamp.nanosec;
        gnss_global.pos << xyz(0), xyz(1), xyz(2);
        gnss_global.vel << 0, 0, 0;

        // Local Position (NED)
        gnss_local.timestamp.sec = msg->header.stamp.sec;
        gnss_local.timestamp.nanosec = msg->header.stamp.nanosec;
        gnss_local.pos << pos_enu(1), pos_enu(0), -pos_enu(2);
        gnss_local.vel << 0, 0, 0;
    }

    void publish_gnss() {
        autorccar_interfaces::msg::Gnss sim_gnss_global; // ECEF
        autorccar_interfaces::msg::Gnss sim_gnss_local;  // NED

        std::random_device rd;
        std::default_random_engine generator(rd());
        std::normal_distribution<double> noise_global_pos(0.0, param.global_position_noise); // (평균, 표준편차)
        std::normal_distribution<double> noise_global_vel(0.0, param.local_velocity_noise);  // (평균, 표준편차)
        std::normal_distribution<double> noise_local_pos(0.0, param.local_position_noise);   // (평균, 표준편차)
        std::normal_distribution<double> noise_local_vel(0.0, param.local_velocity_noise);   // (평균, 표준편차)

        // Global
        sim_gnss_global.timestamp.sec = gnss_global.timestamp.sec;
        sim_gnss_global.timestamp.nanosec = gnss_global.timestamp.nanosec;

        sim_gnss_global.position_ecef.x = gnss_global.pos[0] + noise_global_pos(generator);  // ECEF X
        sim_gnss_global.position_ecef.y = gnss_global.pos[1] + noise_global_pos(generator);  // ECEF Y
        sim_gnss_global.position_ecef.z = gnss_global.pos[2] + noise_global_pos(generator);  // ECEF Z

        sim_gnss_global.velocity_ecef.x = gnss_global.vel[0] + noise_global_vel(generator);
        sim_gnss_global.velocity_ecef.y = gnss_global.vel[1] + noise_global_vel(generator);
        sim_gnss_global.velocity_ecef.z = gnss_global.vel[2] + noise_global_vel(generator);
        
        // Local
        sim_gnss_local.timestamp.sec = gnss_local.timestamp.sec;
        sim_gnss_local.timestamp.nanosec = gnss_local.timestamp.nanosec;

        sim_gnss_local.position_ecef.x = gnss_local.pos[0] + noise_local_pos(generator);  // North
        sim_gnss_local.position_ecef.y = gnss_local.pos[1] + noise_local_pos(generator);  // East
        sim_gnss_local.position_ecef.z = gnss_local.pos[2] + noise_local_pos(generator);  // Down

        sim_gnss_local.velocity_ecef.x = gnss_local.vel[0] + noise_local_vel(generator);
        sim_gnss_local.velocity_ecef.y = gnss_local.vel[1] + noise_local_vel(generator);
        sim_gnss_local.velocity_ecef.z = gnss_local.vel[2] + noise_local_vel(generator);

        pub_gnss_global->publish(sim_gnss_global);
        pub_gnss_local->publish(sim_gnss_local);
    }    
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  rclcpp::spin(std::make_shared<Sensors>(options));
  rclcpp::shutdown();
  return 0;
}