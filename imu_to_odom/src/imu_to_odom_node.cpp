#include "imu_to_odom/imu_to_odom.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuToOdom>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
