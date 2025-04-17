# RDT25-ROBOT

# Run this command before launching aptag_pose_publisher.py so the camera frame is known relative to the robot 
ros2 run tf2_ros static_transform_publisher 0.0 0.0 1.0 0.0 0.0 0.0 camera_link base_link

# Camera Node - ROS2 Example

## Overview

This is a simple ROS2-based camera node that captures video from a webcam using OpenCV and displays the frames in a window. The node continuously captures frames, converts them to ROS `Image` messages, and publishes them to the `april_tag` topic. A subscriber node can then listen to this topic and display the frames in an OpenCV window. The node will continue capturing and publishing frames until the user presses the 'q' key to exit.

## Requirements

To run this example, you need to have the following dependencies installed:

### ROS2:
- **ROS2** (e.g., Foxy, Galactic, or Humble)

### Python:
- **Python 3** (Make sure Python 3 is installed on your system)

### Dependencies:
- **OpenCV** (cv2)
- **ROS2 `rclpy` library**
- **`cv_bridge` for converting between OpenCV and ROS image messages**

To install the required Python dependencies, run the following:

```bash
pip install opencv-python
pip install rospy
```

Additionally, make sure you have `cv_bridge` and ROS2 properly set up in your environment. You can install the necessary ROS2 packages using:

```bash
sudo apt install ros-<ros-distro>-cv-bridge
```
*Replace `<ros-distro>` with your installed ROS2 version (e.g., `foxy`, `galactic`, etc.)*

## Setup

1. **Create a ROS2 Workspace (if not already created)**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. **Create a Package**:
   If you don’t already have a ROS2 package, create one:
   ```bash
   ros2 pkg create --build-type ament_python camera_package
   ```

3. **Add your node (camera_node.py)**:
   - Place your `camera_node.py` script in the `camera_package` directory.

4. **Build the Workspace**:
   Go back to the workspace root and build it:
   ```bash
   cd ~/ros2_ws
   colcon build
   ```

5. **Source the Workspace**:
   Make sure to source the workspace to set up the environment:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

### Starting the Camera Node

1. **Start ROS2 Core (if not already running)**:
   In a new terminal, start the ROS2 core:
   ```bash
   ros2 daemon start
   ```

2. **Run the Camera Publisher Node**:
   In the same terminal where you sourced the workspace, run the camera node:
   ```bash
   ros2 run camera_package camera_node
   ```

   This will start capturing video from the webcam and publishing the frames to the `april_tag` topic.

3. **Run the Subscriber Node (optional)**:
   If you also have a subscriber node (to visualize the frames), you can run it in a separate terminal:
   ```bash
   ros2 run camera_package camera_subscriber
   ```

   This node will display the frames it receives from the `april_tag` topic.

### Exiting the Program

- Press `q` in the OpenCV window to stop the camera node and shut down the ROS2 publisher.

---

## Troubleshooting

- **Camera not working**: If the camera doesn't open, ensure that the webcam is properly connected and available. You can check if the camera is working by testing it with OpenCV outside of ROS2.
  
- **ROS2 Topic not showing data**: Make sure there is a subscriber node running and that it's correctly subscribing to the `april_tag` topic. You can check this using:
  ```bash
  ros2 topic list
  ros2 topic echo /april_tag
  ```

---

## Summary

This project demonstrates how to use OpenCV to capture video frames, convert them to ROS image messages, and publish them to a topic. A subscriber node can then listen to that topic, convert the ROS image messages back into OpenCV format, and display the frames.

This is a simple ROS2 example that can be extended to incorporate more advanced functionalities like camera calibration, image processing, and more.


