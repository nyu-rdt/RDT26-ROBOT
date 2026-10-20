contains the URDF for NYU RDT's 2026 robot so that all can refer to one urdf and all changes can be made at once for all pkgs.

![The robot in RViz with joint_state_publisher running](docs/rviz.png)

# Launching the URDF

Run everything inside the ROS container, from the workspace root.

## Build

```bash
cd ~/rdt26/rdt26-robot
colcon build --packages-select nyurdt_description nyurdt_navigation
source install/setup.bash
```

## Pick ONE joint publisher

The wheels are `continuous` joints. Their frames only exist while something
publishes `/joint_states`. If nothing does, RViz draws all four wheels piled
up at the origin. Run exactly one of the options below, never two at once.

### 1. Static model (no moving parts)

```bash
ros2 launch nyurdt_description rsp.launch.py
ros2 run joint_state_publisher joint_state_publisher   # second terminal
rviz2 -d $(ros2 pkg prefix --share nyurdt_description)/rviz/robot.rviz   # third terminal
```

### 2. Interactive model (sliders spin the wheels)

```bash
ros2 launch nyurdt_description rsp.launch.py
ros2 run joint_state_publisher_gui joint_state_publisher_gui   # second terminal
rviz2 -d $(ros2 pkg prefix --share nyurdt_description)/rviz/robot.rviz   # third terminal
```

If the GUI is missing: `apt install ros-jazzy-joint-state-publisher-gui`

### 3. Gazebo simulation (Gazebo publishes the joints)

```bash
ros2 launch nyurdt_navigation gazebo.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # second terminal, drive with i j k l
rviz2 -d $(ros2 pkg prefix --share nyurdt_description)/rviz/robot.rviz   # optional, third terminal
```

Do not run `joint_state_publisher` here. Gazebo's JointStatePublisher plugin
already publishes the real wheel angles, and `rsp.launch.py` is started by the
launch file for you.

## Viewing the robot in Gazebo

`gazebo.launch.py` opens the Gazebo window with `my_world.sdf`, spawns the
robot as `rdt26_robot`, and bridges its topics to ROS. The mesh paths are set
by the launch file, so the full model (wheels, excavator, hopper) should load.

### Moving the Gazebo camera

| Action | Mouse |
|---|---|
| Pan | left-click drag |
| Orbit | middle-click drag, or Shift + left-click drag |
| Zoom | scroll, or right-click drag |
| Follow the robot | right-click the robot, then Follow |
| Jump to the robot | right-click the robot, then Move to |

The robot is also listed in the Entity Tree panel on the right.

### Driving it

With `teleop_twist_keyboard` running, click its terminal so it has focus:
`i` forward, `,` back, `j` / `l` turn, `k` stop, `q` / `z` change speed.
Make sure the play button at the bottom left of Gazebo is not paused.

### Seeing what the RealSense sees

```bash
ros2 run rqt_image_view rqt_image_view   # pick /camera/image_raw from the dropdown
```

In Gazebo itself: the top-right menu (three dots), then Image Display, then
pick `realsense/image`.

### Checking it from a terminal

```bash
ros2 topic hz /odom               # about 50 Hz: wheels are being simulated
ros2 topic hz /camera/image_raw   # about 28 Hz: camera is rendering
ros2 topic echo /joint_states --once
```

| ROS topic | What it is |
|---|---|
| `/cmd_vel` | velocity commands in (teleop, Nav2) |
| `/odom` | wheel odometry out |
| `/joint_states` | wheel angles out |
| `/camera/image_raw`, `/camera/depth/image_rect_raw`, `/camera/points` | RealSense color, depth, point cloud |
| `/clock` | sim time |

### Stopping it

Ctrl+C in the launch terminal. If Gazebo stays open:
`pkill -f "gz sim"`

## RViz config

Always start RViz with `-d .../rviz/robot.rviz` as above. It shows the robot
model and its TF frames with Fixed Frame `base_link`. Plain `rviz2` opens
with Fixed Frame `map`, which nothing publishes here, so the robot never
appears.

In Gazebo, change Fixed Frame to `odom` in Global Options to watch the robot
drive instead of staying centred.

## Updating the meshes from CAD

The meshes in `meshes/` come from the Fusion export. Run on a machine with
Blender, then copy `body.*`, `wheel.*` and `frames.txt` into `meshes/` and
paste the numbers from `frames.txt` into the properties at the top of
`urdf/robot.xacro`.

```bash
blender -b --python scripts/glb_to_meshes.py -- rover26.glb out_dir 100000 10000
```
