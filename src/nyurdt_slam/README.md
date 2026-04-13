To run the realsense camera here are the options:
ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 gyro_fps:=400 accel_fps:=200 enable_infra1:=false enable_infra2:=false enable_sync:=false align_depth.enable:=true pointcloud.enable:=false initial_reset:=true depth_module.enable_auto_exposure:=true decimation_filter.enable:=false temporal_filter.enable:=true spatial_filter.enable:=true

* ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=true 

ros2 launch rtabmap_examples realsense_d435i_stereo.launch.py

ros2 launch realsense2_camera rs_launch.py enale_infra1:=true enable_infra2:=true enable_sync:=true 

ros2 launch realsense2_camera rs_launch.py enable_infra1:=true enable_infra2:=true unite_imu_method:=linear_interpolation enable_gyro:=true enable_accel:=true enable_sync:=true pointcloud.enable:=true

 ros2 launch realsense2_camera rs_launch.py enable_infra1:=true enable_infra2:=true unite_imu_method:=linear_interpolation enable_gyro:=true enable_accel:=true enable_sync:=true

To launch rtabmap in a separate terminal:

*  ros2 launch rtabmap_launch rtabmap.launch.py  args:="--delete_db_on_start"  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw  rgb_topic:=/camera/camera/color/image_raw  camera_info_topic:=/camera/camera/color/camera_info  approx_sync:=true  frame_id:=camera_link queue_size:=100 topic_queue_size:=100

ros2 launch rtabmap_launch rtabmap.launch.py  args:="--delete_db_on_start" left_image_topic:=/camera/camera/infra1/image_rect_raw left_camera_info:=/camera/camera/infra1/camera_info imu_topic:=/camera/camera/accel/imu_info

ros2 launch rtabmap_launch rtabmap.launch.py  args:="--delete_db_on_start"  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw  rgb_topic:=/camera/camera/color/image_raw  camera_info_topic:=/camera/camera/color/camera_info  approx_sync:=true  frame_id:=camera_link queue_size:=100 topic_queue_size:=100

ros2 launch rtabmap_launch rtabmap.launch.py  args:="--delete_db_on_start" left_image_topic:=/camera/camera/infra1/image_rect_raw left_camera_info:=/camera/camera/infra1/camera_info imu_topic:=/camera/camera/accel/imu_info

ros2 launch rtabmap_launch rtabmap.launch.py  args:="--delete_db_on_start"  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw  rgb_topic:=/camera/camera/color/image_raw  camera_info_topic:=/camera/camera/color/camera_info  approx_sync:=true  frame_id:=camera_link queue_size:=100 topic_queue_size:=100


### LAUNCH USING YAML FILE
Main launch:
ros2 launch src/launch/main.launch.yaml

This calls them in order. First, the camera turns os, and second, the rtabmap gui opens 