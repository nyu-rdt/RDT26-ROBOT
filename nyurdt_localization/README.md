1. connect lidar to the port (default port: /dev/ttyUSB0)
2. run `bash run_lidar.sh` (this may need you to enter the password of your device since it requires root authority to activate the port)
3. The output can be visualized on rviz2:\
run `ros2 run rviz2 rviz2` and add 'lidar_scan' PointCloud2 topic.