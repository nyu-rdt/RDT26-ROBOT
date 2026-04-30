from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_apriltag = LaunchConfiguration("use_apriltag")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_rviz = LaunchConfiguration("use_rviz")

    rtabmap_params = PathJoinSubstitution([
        FindPackageShare("rdt_nav2_bringup"),
        "config",
        "rtabmap_params.yaml"
    ])

    nav2_params = PathJoinSubstitution([
        FindPackageShare("rdt_nav2_bringup"),
        "config",
        "nav2_params.yaml"
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_apriltag", default_value="false"),
        DeclareLaunchArgument("use_nav2", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),

        # RealSense camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("realsense2_camera"),
                    "launch",
                    "rs_launch.py"
                ])
            ]),
            launch_arguments={
                "enable_rgbd": "true",
                "enable_sync": "true",
                "align_depth.enable": "true",
                "depth_module.profile": "640x480x30",
                "rgb_camera.profile": "640x480x30",
                "pointcloud.enable": "true",
            }.items()
        ),

        # RGB-D odometry
        Node(
            package="rtabmap_odom",
            executable="rgbd_odometry",
            name="rgbd_odometry",
            output="screen",
            parameters=[{
                "frame_id": "base_link",
                "odom_frame_id": "odom",
                "publish_tf": True,
                "approx_sync": True,
                "wait_for_transform": 0.2,
            }],
            remappings=[
                ("rgb/image", "/camera/camera/color/image_raw"),
                ("depth/image", "/camera/camera/aligned_depth_to_color/image_raw"),
                ("rgb/camera_info", "/camera/camera/color/camera_info"),
            ]
        ),

        # RTAB-Map SLAM/localization
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=[
                rtabmap_params,
                {
                    "frame_id": "base_link",
                    "odom_frame_id": "odom",
                    "map_frame_id": "map",
                    "subscribe_rgbd": False,
                    "subscribe_depth": True,
                    "approx_sync": True,
                    "publish_tf": True,
                }
            ],
            remappings=[
                ("rgb/image", "/camera/camera/color/image_raw"),
                ("depth/image", "/camera/camera/aligned_depth_to_color/image_raw"),
                ("rgb/camera_info", "/camera/camera/color/camera_info"),
                ("odom", "/odom"),
            ]
        ),

        # Optional AprilTag support
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("rdt_nav2_bringup"),
                    "launch",
                    "apriltag.launch.py"
                ])
            ]),
            condition=IfCondition(use_apriltag)
        ),

        # Optional real Nav2 launch, not sim launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "navigation_launch.py"
                ])
            ]),
            launch_arguments={
                "params_file": nav2_params,
                "use_sim_time": "false",
            }.items(),
            condition=IfCondition(use_nav2)
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(use_rviz),
        ),
    ])from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_apriltag = LaunchConfiguration("use_apriltag")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_rviz = LaunchConfiguration("use_rviz")

    rtabmap_params = PathJoinSubstitution([
        FindPackageShare("nyurdt_bringup"),
        "config",
        "rtabmap_params.yaml"
    ])

    nav2_params = PathJoinSubstitution([
        FindPackageShare("nyurdt_bringup"),
        "config",
        "nav2_params.yaml"
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_apriltag", default_value="false"),
        DeclareLaunchArgument("use_nav2", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),

        # RealSense camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("realsense2_camera"),
                    "launch",
                    "rs_launch.py"
                ])
            ]),
            launch_arguments={
                "enable_rgbd": "true",
                "enable_sync": "true",
                "align_depth.enable": "true",
                "depth_module.profile": "640x480x30",
                "rgb_camera.profile": "640x480x30",
                "pointcloud.enable": "true",
            }.items()
        ),

        # RGB-D odometry
        Node(
            package="rtabmap_odom",
            executable="rgbd_odometry",
            name="rgbd_odometry",
            output="screen",
            parameters=[{
                "frame_id": "base_link",
                "odom_frame_id": "odom",
                "publish_tf": True,
                "approx_sync": True,
                "wait_for_transform": 0.2,
            }],
            remappings=[
                ("rgb/image", "/camera/camera/color/image_raw"),
                ("depth/image", "/camera/camera/aligned_depth_to_color/image_raw"),
                ("rgb/camera_info", "/camera/camera/color/camera_info"),
            ]
        ),

        # RTAB-Map SLAM/localization
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=[
                rtabmap_params,
                {
                    "frame_id": "base_link",
                    "odom_frame_id": "odom",
                    "map_frame_id": "map",
                    "subscribe_rgbd": False,
                    "subscribe_depth": True,
                    "approx_sync": True,
                    "publish_tf": True,
                }
            ],
            remappings=[
                ("rgb/image", "/camera/camera/color/image_raw"),
                ("depth/image", "/camera/camera/aligned_depth_to_color/image_raw"),
                ("rgb/camera_info", "/camera/camera/color/camera_info"),
                ("odom", "/odom"),
            ]
        ),

        # Optional AprilTag support
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("rdt_nav2_bringup"),
                    "launch",
                    "apriltag.launch.py"
                ])
            ]),
            condition=IfCondition(use_apriltag)
        ),

        # Optional real Nav2 launch, not sim launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "navigation_launch.py"
                ])
            ]),
            launch_arguments={
                "params_file": nav2_params,
                "use_sim_time": "false",
            }.items(),
            condition=IfCondition(use_nav2)
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(use_rviz),
        ),
    ])