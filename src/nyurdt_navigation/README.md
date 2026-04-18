# 🤖 Robot Visualization Guide
Quick guide for exposing our robot to **RViz**.

We will use a single `launch.py` file which will:
1.  **Start Gazebo simulation**
2.  **Start RViz simulation**
3.  **Publish the state** of our robot by extracting the provided `.xacro` file.

*Before the end of the guide, we will also export the respective TF tree of the robot.*

---

## A. Initial Setup

```bash
cd ~/RDT/RDT26-ROBOT
colcon build --packages-select nyurdt_navigation --symlink-install
source install/setup.bash
```

## B. Launch Gazebo and RVIZ and Robot State Publisher

```bash
ros2 launch nyurdt_navigation nav2_sim.launch.py
```

## C. Export the TF tree

```bash
ros2 run tf2_tools view_frames
```

## D. Configuration inside the RVIZ

1. Check the RobotModel box (in the DIsplays section - up and left)
2. Fixed Frame = base_footprint (in the DIsplays section - up and left)

**Next Move => Expose the map (not only the robot) in order for the actual part (localization and navigation) of NAV2 to take place.**
