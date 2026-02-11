# AprilTag Setup
## In a new terminal (in workspace src folder):
1. source /opt/ros/jazzy/setup.bash
2. colcon build for latest changes
3. source install/setup.bash (isb shortcut on NUC)
4. cd into RDT26-ROBOT
5. git checkout apriltag (make sure you are in branch)
## Running:
ros2 launch april launch_robot.launch.py (for everything)
-- or --
ros2 launch april camera_apriltag.launch.py (if just checking to see if camera loads and is detecting apriltags)
(note: make sure to change file path next to camera_info_url parameter in the launch file if running on own computer)

To visualize everything:
1. Open new terminal tab 
2. Source your files (src folder)
3. rviz2 (for now)




