import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration 

def generate_launch_description():


    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    navigation   = get_package_share_directory('nyurdt_navigation')
    description  = get_package_share_directory('nyurdt_description')
    realsense    = get_package_share_directory('realsense2_camera')
    localization = get_package_share_directory('nyurdt_localization')
    slam         = get_package_share_directory('nyurdt_slam')

    
    nav2_params = os.path.join(navigation, 'config', 'nav2_params.yaml')
    realsense_params = os.path.join(slam, 'config', 'realsense.yaml')

    return LaunchDescription([
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # ── Robot State Publisher (from nyurdt_description) ──────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description, 'launch', 'rsp.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # ── RealSense ────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(realsense, 'launch', 'rs_launch.py')
            ),
            launch_arguments={
                # 'config_file': realsense_params,
                    # ── Core streams ─────────────────────────────────────────────
                "align_depth.enable": "true",
                "pointcloud.enable": "true",
                "enable_infra1": "true",
                "enable_infra2": "true",
                "enable_sync": "true",

                # ── Depth / IR profiles ──────────────────────────────────────
                "depth_module.depth_profile": "480x270x90",
                "depth_module.infra_profile": "480x270x90",
                "rgb_camera.color_profile":   "424x240x90",

                # "unite_imu_method": "2",

                # ── IMU ──────────────────────────────────────────────────────
                "enable_gyro": "true",
                "enable_accel": "true",
                # "imu_topic": "/imu/data",
                # initial_reset: true

                # ── Decimation filter ────────────────────────────────────────
                "decimation_filter.enable": "true",
                "decimation_filter.filter_magnitude": "2",

                # ── Transport (avoid re-encoding overhead) ───────────────────
                "rgb_transport": "raw",
                "depth_transport": "raw",
                "colorizer.enable": "false",

                # ── Latency / queue tuning ───────────────────────────────────
                "depth_module.frames_queue_size": "5",
                "rgb_camera.frames_queue_size":   "5",

                # ── Spatial filter (smooth depth) ────────────────────────────
                "spatial_filter.enable": "true",
                "spatial_filter.frames_queue_size": "5",

                # ── Temporal filter (reduce flickering) ──────────────────────
                "temporal_filter.enable": "true",
                "temporal_filter.frames_queue_size": "5",

                # ── Disabled filters ─────────────────────────────────────────
                "hole_filling_filter.enable": "false",
                "disparity_filter.enable":    "false",
                "hdr_merge.enable":           "false",

                # ── Clip distance (indoor stability) ─────────────────────────
                "clip_distance": "5.0",
            }.items()
        ),

        # ── Apriltag ────────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(localization, 'launch', 'camera_apriltag.launch.py'),
            ),
            launch_arguments={'device2': '0'}.items() 
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam, 'launch', 'main.launch.py')
            ),
        ),

        # ── Nav2 (from nyurdt_navigation) ─────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(navigation, 'launch', 'nav2_standalone.launch.py')
            ),
            launch_arguments={
                'params_file': nav2_params,
                'use_sim_time': use_sim_time,
            }.items()
        ),

        # ── Autonomy Nodes ───────────────────────────────────────────────
        Node(package='nyurdt_control', executable='i2c_reader', name='i2c_reader'),
        Node(package='nyurdt_autonomy', executable='telemetry_processor', name='telemetry_processor'),
        Node(package='nyurdt_autonomy', executable='autonomy_manager', name='autonomy_manager'),
    ])