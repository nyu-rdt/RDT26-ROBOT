import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():


    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    navigation   = get_package_share_directory('nyurdt_navigation')
    description  = get_package_share_directory('nyurdt_description')
    realsense    = get_package_share_directory('realsense2_camera')
    localization = get_package_share_directory('nyurdt_localization')
    slam         = get_package_share_directory('nyurdt_slam')
    control      = get_package_share_directory('nyurdt_control')

    
    nav2_params = os.path.join(navigation, 'config', 'nav2_params.yaml')
    launch_rviz = LaunchConfiguration('rviz', default='false')
    map_yaml = LaunchConfiguration('map', default=os.path.join(
    get_package_share_directory('nyurdt_navigation'), 'config', 'arena_map.yaml'))

    return LaunchDescription([

        # Add to LaunchDescription
        DeclareLaunchArgument('map', 
            default_value=os.path.join(navigation, 'config', 'arena_map.yaml'),
            description='Full path to map yaml'),

        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # ── Robot State Publisher ─────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description, 'launch', 'rsp.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # ── Joint State Publisher (fixes wheel TFs) ───────────────────────
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'use_sim_time': use_sim_time,
                'publish_default_velocities': True,
                'publish_default_efforts': True,
            }],
        ),
        
        # ── Sim RealSense camera bridge (Gazebo → ROS) ───────────────────
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="camera_bridge",
            arguments=[
                # Depth image
                "/intel_realsense_r200_depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
                # Color image  
                "/intel_realsense_r200_depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                # Point cloud
                "/intel_realsense_r200_depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            ],
            output="screen",
        ),


        # # ── Apriltag ────────────────────────────────────────────────────
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(localization, 'launch', 'camera_apriltag.launch.py'),
        #     ),
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam, 'launch', 'main.launch.py')
            ),
            launch_arguments={
                "args": "--delete_db_on_start",
                "depth_topic":       "/camera/camera/aligned_depth_to_color/image_raw",
                "rgb_topic":         "/camera/camera/color/image_raw",
                "camera_info_topic": "/camera/camera/color/camera_info",

                # Stop RTABMap driving the robot
                "visual_odometry": "false",
                "publish_tf_odom": "false",
                "publish_tf_map": "true",
                "initial_pose": "8.10 3.57 0.0 0.0 0.0 0.0",
                
                "approx_sync": "true",
                "frame_id": "base_link",
                "sync_queue_size": "10",
                "topic_queue_size": "10",
                "subscribe_occupancy_grid": "true",
                "map_topic": "/static_map",
                # * BEST PARAM, DO NOT TOUCH, FAST AND ACCURATE!!!
                "PyDetector/Cuda": "false",
                # * BEST PARAM, DO NOT TOUCH, FAST AND ACCURATE!!!
                "PyMatcher/Cuda": "false",
                # * BEST PARAM, DO NOT TOUCH, FAST AND ACCURATE!!!
                "SuperPoint/Cuda": "false",
                # * BEST PARAM, DO NOT TOUCH, FAST AND ACCURATE!!! | ! NEVER SET TO TRUE
                "Grid/RayTracing": "false",


                # ! HORRIBLE PARAMETER SETTING, DO MORE TESTING
                # "Odom/Strategy": "1",

                # * GOOD PARAM, DO NOT TOUCH!
                # "Vis/CorType": "1",

                # ! HORRIBLE PARAMETER SETTING, DO NOT USE
                # "Odom/ImageDecimation": "2",

                # * BEST PARAM, DO NOT TOUCH, FAST AND ACURATE!!!
                # "Odom/VisKeyFrameThr": "500",

                # ~ GOOD PARAM, DO MORE TESTING
                "OdomF2M/MaxSize": "1000",

                # ! HORRIBLE PARAMETER SETTING, DO NOT USE, USE DEFAULT INSTEAD (1)
                # "OdomF2M/BundleAdjustment": "0",

                # ~ GOOD PARAM, DO MORE TESTING
                # "Vis/MaxFeatures": "800",

                # ~ GOOD PARAM, DO MORE TESTING, LOWER INCREASES RISK OF LOST TRACKING, BUT RUNS FASTER
                "Vis/MinInliers": "15",

                # ~ GOOD PARAM, DO MORE TESTING, LOWER INCREASES RISK OF LOST TRACKING
                # "Vis/Iterations": "200",

                # * GOOD PARAM, DO NOT TOUCH, USING DEFAULT IS BEST (20)
                "FAST/Threshold": "10",

                # * BEST PARAM, DO NOT TOUCH, FAST AND ACURATE!!!
                "Rtabmap/DetectionRate": "1",

                # ~ GOOD PARAM, DO MORE TESTING, UNSURE HOW MUCH IT HELPS
                "Rtabmap/TimeThr": "700",

                # ~ GOOD PARAM, DO MORE TESTING, UNSURE HOW MUCH IT HELPS
                "RGBD/LinearUpdate": ".3",

                # ~ GOOD PARAM, DO MORE TESTING, UNSURE HOW MUCH IT HELPS
                "RGBD/AngularUpdate": ".3",

                # ~ 
                "Rtabmap/MemoryThr": "0",

                # ^ Use default for now (500)
                # ~ GOOD PARAM, DO MORE TESTING, LOWER MEANS LESS POINTS TO REF, MORE MEANS MORE POINTS TO REF. MORE POINTS MEANS MORE COMPUTATION, BUT ALSO MORE ACCURATE
                "Kp/MaxFeatures": "600",

                # TODO: DO MORE TESTING, DID NOT SEE MUCH DIFFERENCE IN TESTING
                "Mem/STMSize": "1000",

                # TODO: DO MORE TESTING, DID NOT SEE MUCH DIFFERENCE IN TESTING
                "Mem/RecentWmRatio": "0.2",

                # * BEST PARAM, DO NOT TOUCH, FAST AND ACURATE!!!
                "Grid/DepthDecimation": "4",

                # ~ GOOD PARAM, DO MORE TESTING, UNSURE HOW MUCH IT HELPS
                # "Grid/CellSize": "0.07",

                # ! HORRIBLE PARAMETER SETTING, DO NOT USE, USE DEFAULT INSTEAD (1)
                # "Grid/NoiseFilteringRadius": "0.05",

                # TODO: DO MORE TESTING
                # "Grid/NoiseFilteringMinNeighbors": "5",

                # * GOOD PARAM, DO NOT TOUCH, USING DEFAULT IS BEST (true)
                # "Grid/NormalsSegmentation": "true",

                # ~ GOOD PARAM, DO MORE TESTING, UNSURE HOW MUCH IT HELPS
                # "Grid/MaxGroundAngle": "30",

                # * GOOD PARAM, DO NOT TOUCH, FAST AND ACURATE!!! USE DEFAULT (20)
                # "Optimizer/Iterations": "20",

                # * BEST PARAM, DO NOT TOUCH, FAST AND ACURATE!!! USE DEFAULT (true)
                "RGBD/CreateOccupancyGrid": "true",

                # * BEST PARAM, DO NOT TOUCH, FAST AND ACURATE!!!
                "Grid/3D": "false",

                # ! HORRIBLE PARAMETER SETTING, DO NOT MAKE TRUE (TAKES WALLS OUT OF EVERYTHING)
                # "Grid/GroundIsObstacle": "true",

                # * GOOD PARAM, DO NOT TOUCH, USING DEFAULT IS BEST (true)
                "Grid/FlatObstacleDetected": "true",

                # "Grid/MapFrameProjection": "true",

                # "Grid/MaxObstacleHeight": "2.5",

                # "Grid/MinGroundHeight": "-0.25",

                # "Grid/MaxGroundHeight": "0.25",

                # "Mem/ImagePreDecimation": "2",

                # * BEST PARAM, DO NOT TOUCH, PREVENTS SWAP MEMORY FROM GROWING TOO MUCH
                "Odom/ScanKeyFrameThr": ".6",

                # "always_process_most_recent_frame": "false",
            }.items()
        ),

        # ── Nav2 (from nyurdt_navigation) ─────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(navigation, 'launch', 'nav2_sim.launch.py')
            ),
            launch_arguments={
                'params_file': nav2_params,
                'use_sim_time': use_sim_time,
                 'map': map_yaml,  
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(control, 'launch', 'control.launch.py')
            ),
        ),

        # ── Static TFs ───────────────────────────────────────────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']
        ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='odom_to_base',
        #     arguments=['8.10', '3.57', '0.0', '0.0', '0.0', '0.0', 'odom', 'base_link']
        # ),


        Node(
            package='rviz2',
            executable='rviz2',
            condition=IfCondition(launch_rviz)
        ),

        # ── Autonomy Nodes ───────────────────────────────────────────────
        Node(package='nyurdt_control', executable='i2c_reader', name='i2c_reader'),
        Node(package='nyurdt_autonomy', executable='telemetry_processor', name='telemetry_processor'),
        Node(package='nyurdt_autonomy', executable='autonomy_manager', name='autonomy_manager'),
        
    ])
