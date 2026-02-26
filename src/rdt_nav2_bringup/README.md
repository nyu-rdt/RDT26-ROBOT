Quick guide for exposing our robot to Gazebo and RVIZ
We will use only one joint 'launch.py' file which will :
1. start Gazebo simulation
2. start RVIZ simulation
3. publish the state of our robot by extracting the provided '.xacro' file
Before the end of the guide we will also export the respective TF tree of the robot.

A. Initial Setup
cd ~/RDT/RDT26-ROBOT
colcon build --packages-select rdt_nav2_bringup --symlink-install
source install/setup.bash

B. Launch Gazebo and RVIZ and Robot State Publisher
ros2 launch rdt_nav2_bringup nav2_sim.launch.py
Export the TF tree
ros2 run tf2_tools view_frames

C. Configuration inside the RVIZ
Check the RobotModel box (in the DIsplays section - up and left)
Fixed Frame = base_footprint (in the DIsplays section - up and left)

Next Move => Expose the map (not only the robot) in order for the actual part (localization and navigation) of NAV2 to take place.