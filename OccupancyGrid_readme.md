PointCloud to Laserscan

A) Initialization:
After installing ros2 humble enter the following command in every terminal to set the source of the terminal to ros2 humble:
source /opt/ros/humble/setup.bash

Requirements:
1) realsense ros wrapper
   Installation: https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file
   1. configure ubuntu libraries:
   	a) run: 
   	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   	b) run:
   	sudo apt install curl # if you haven't already installed curl
      curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

   2. Install realsense sdk: 
   	sudo apt install ros-humble-librealsense2*
   3. Install realsense ros wrapper:
   	sudo apt install ros-humble-realsense2-*
   	
2) pointcloud_to_laserscan
   Install using: sudo apt-get install ros-humble-pointcloud-to-laserscan
   
3) slam_toolbox
   Install using: sudo apt install ros-humble-slam-toolbox

B) Launching:
Remember to run: source /opt/ros/humble/setup.bash to set source of each terminal.

1) Run in terminal 1 to start publishing pointclouds in /camera/camera/depth/color/points topic:
   ros2 launch realsense2_camera rs_launch.py filters:=pointcloud align_depth:=true pointcloud.enable:=true enable_gyro:=true enable_accel:=true

2) Run in terminal 2 to launch rviz and subscribe to pointcloud in /camera/camera/depth/color/points:
   rviz2

3) Run in terminal 3 to launch the pointcloud to laserscan script:
   ros2 launch launch_latest_pc2ls.py

4) Now in rviz you can subscribe to /scanner/scan to see the laserscan. The pointcloud needs to remain subscribed as the pointcloud to 
   laserscan script require one other subscriber to the pointcloud. So can turn the alpha value of the pointcloud to 0 to make it invisible 
   and see the laserscan.   
   
------------------------------------------------------------------------------------------------------------------------------------------------------------  
BNO085 IMU Setup with Raspberry pi 4

Sensor Connections:

Sensor - Raspberry pi 4
 Vcc   - Raspberry pi 3.3v pin
 Grd   - Raspberry pi ground pin
 SCL   - GPIO24/Pin 18 (for software i2c)
 SDA   - GPIO23/Pin 16 (for software i2c)

1) Download the ROS2 driver from the git repo: https://github.com/robert-stevenson-1/BNO085-ROS2-Node.git

2) For Raspberry pi 4 you will need to enable software I2c to communicate over the I2c protocol with the IMU. For this follow the instructions on this page: https://github.com/fivdi/i2c-bus/blob/master/doc/raspberry-pi-software-i2c.md

One change to this is, since we are using Ubuntu on raspberry pi instead of Raspbian, the /boot/config.txt file is actually in /boot/firmware/config.txt.

3) One of the requirements to run the Driver is this library: https://github.com/adafruit/Adafruit_CircuitPython_BNO08x.git

In this navigate to the file after downloading this: /python_directory/adafruit_bno08x/i2c.py 
and change the default i2c address of the BNO085 on line 13 from 0x4A to 0x4B as this is the default address of our sensor for whatever reason.

4) Once this setup and the necessary libraries detailed in the ROS driver git have been installed, run python3 src/bno085/bno085/BNO085_pub.py from the BNO085-ROS2-Node folder.
For Raspberry pi 5, if using hardware i2c, channge the I2c bus being used (normally 1) in line 43: i2c = I2C(1)
bus 3 is for software i2c.

5) Also in the terminal running the python publisher, run chmod 666 /dev/i2c-3 to give the terminal permission to access the i2c bus.

6) This should lead to the sensor publishing under the IMU_Data topic. 
In some cases if the sensor does not connect, might need to reset the sensor. To do this, provide a quick high pulse on the RST/reset pin of the sensor. This can be done using a jumper wire to quickly connect and disconnect the pin to the 3.3v supply.

**TBD: 
1) Connect the raspberry pi ros topic to the NUC ros2 using HTTP
   Guides: https://husarion.com/tutorials/ros2-tutorials/6-robot-network/
   	   https://roboticsbackend.com/ros2-multiple-machines-including-raspberry-pi/
2) Convert the IMU_Data to Odometry (albeit inaccurately due to IMu drift)
   github repo: https://github.com/nadiawangberg/imu_to_odom/tree/master
   

------------------------------------------------------------------------------------------------------------------------------------------------------------

Running Slam-Toolbox (Finally)
   
Once the TBD items are complete, we can run all the above packages to start publishing the laserscan and odom and then we can run the following in separate terminals:

ros2 launch nav2_bringup navigation_launch.py

ros2 launch slam_toolbox online_async_launch.py

ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz

This should start mapping the area and we can visualize this is rviz.

------------------------------------------------------------------------------------------------------------------------------------------------------------





