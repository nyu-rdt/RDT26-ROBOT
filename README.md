# RDT25-ROBOT

## HOW TO RUN STATE MACHINES

### Have two separate terminal windows open

### In one of them:
```bash
cd ~/Desktop/rdt25_gcs_ws
colcon build
isb # source workspace alias shortcut
ros2 run joy joy_node
```

In another terminal in the same window
```bash
isb
ros2 run nyurdt_gcs_manual_control joystick_main 
```

In third terminal in the same window
```bash
isb
ros2 launch nyurdt_gazebo_sim launch_sim.launch.py
```

### In the second terminal window
```bash
cd ~/Desktop/rdt25_robot_ws
colcon build
isb # source global ros
ros2 run GCS_command_subscriber
```

Press the button with two overlapping rectangles to start locomotion autonomy

