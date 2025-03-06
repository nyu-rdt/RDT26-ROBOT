#ifndef IMU_TO_ODOM_H
#define IMU_TO_ODOM_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Dense>

class ImuToOdom : public rclcpp::Node {
public:
    ImuToOdom();
    ~ImuToOdom();

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    
    Eigen::Vector3d position_;
    Eigen::Quaterniond orientation_;
    Eigen::Vector3d velocity_;
    
    rclcpp::Time last_time_;
    rclcpp::Clock::SharedPtr clock_;
};

#endif // IMU_TO_ODOM_H

