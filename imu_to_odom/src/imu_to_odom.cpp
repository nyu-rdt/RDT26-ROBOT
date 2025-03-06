#include "imu_to_odom/imu_to_odom.hpp"


ImuToOdom::ImuToOdom() : Node("imu_to_odom"), position_(0, 0, 0), velocity_(0, 0, 0), orientation_(Eigen::Quaterniond::Identity()) {
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "IMU_Data", 10, std::bind(&ImuToOdom::imuCallback, this, std::placeholders::_1));
    
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    
    last_time_ = this->get_clock()->now();
}

ImuToOdom::~ImuToOdom() {}

void ImuToOdom::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    rclcpp::Time current_time = this->get_clock()->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;
    
    // Integrate IMU data to estimate position
    Eigen::Vector3d acceleration(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    velocity_ += acceleration * dt;
    position_ += velocity_ * dt;
    
    orientation_ = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    
    // Publish odometry message
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.pose.pose.position.x = position_.x();
    odom_msg.pose.pose.position.y = position_.y();
    odom_msg.pose.pose.position.z = position_.z();
    odom_msg.pose.pose.orientation.w = orientation_.w();
    odom_msg.pose.pose.orientation.x = orientation_.x();
    odom_msg.pose.pose.orientation.y = orientation_.y();
    odom_msg.pose.pose.orientation.z = orientation_.z();
    
    odom_pub_->publish(odom_msg);
}

