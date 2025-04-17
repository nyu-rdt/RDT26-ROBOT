# RDT25-ROBOT

NOTE: this will not work on the NUC as of now because it is using numpy 1.21 instead of numpy 2.x

how to run april tag publisher and subscriber:

in one terminal:
cd rdt25_gcs_ws
colcon build
. install/setup.bash
ros2 run camera_pkg camera_node

in another terminal:
. install/setup.bash
ros2 run camera_pkg camera_sub

in another terminal:
. install/setup.bash
ros2 run camera_pkg tag_sub