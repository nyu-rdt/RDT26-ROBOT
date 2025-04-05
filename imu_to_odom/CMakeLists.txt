cmake_minimum_required(VERSION 3.5)
project(imu_to_odom)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_executable(imu_to_odom_node src/imu_to_odom_node.cpp src/imu_to_odom.cpp)
ament_target_dependencies(imu_to_odom_node rclcpp nav_msgs sensor_msgs geometry_msgs Eigen3)

install(TARGETS imu_to_odom_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()

