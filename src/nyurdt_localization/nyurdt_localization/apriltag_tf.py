'''
apriltag_tf.py
'''
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, LookupException, ConnectivityException, ExtrapolationException
import tf_transformations as ts

# Known tag poses in MAP frame:
# (x, y, z, roll, pitch, yaw) in metres/radians
TAG_POSES_IN_MAP = {
    'tag_18': (0.0, 0.0, 0.0, 0.0, 1.57, 0.0),
    'tag_19': (0.0, 0.0, 0.0, 0.0, 1.57, 0.0),
    'tag_20': (0.0, 0.0, 0.0, 0.0, 1.57, 0.0),
    'tag_21': (0.0, 0.0, 0.0, 0.0, 1.57, 0.0),
}

class AprilTagLocalizationNode(Node):
    def __init__(self):
        super().__init__('apriltag_localization')

        # Create instance of buffer and listener to subscribe to /tf topic and store transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/apriltag/robot_pose',
            10
        )

        # Covariance to assign to apriltag-derived pose corrections
        # Tune these — smaller = more trust in tag, less in odometry
        self.position_variance = 0.02 # metres^2
        self.orientation_variance = 0.01 # radians^2

        self.timer = self.create_timer(0.1, self._process_frames)

    def _process_frames(self):
        for tag_id, map_pose in TAG_POSES_IN_MAP.items():
            try:
                # ── tag -> camera transform
                tag_to_cam_tf = self.tf_buffer.lookup_transform(
                    'camera_optical_frame_link',
                    tag_id,
                    Time()
                )

                self.get_logger().info(f"Tag {tag_id} detected at distance {tag_to_cam_tf.transform.translation.z:.2f}m", throttle_duration_sec=0.5)
                t = tag_to_cam_tf.transform.translation
                q = tag_to_cam_tf.transform.rotation

                trans = ts.translation_matrix([t.x, t.y, t.z])
                rot = ts.quaternion_matrix([q.x, q.y, q.z, q.w])

                T_tag_to_cam = ts.concatenate_matrices(trans, rot)

                # ── map→tag (from known arena layout) ───────────────────────
                x, y, z, roll, pitch, yaw = map_pose
                T_map_to_tag = ts.concatenate_matrices(
                ts.translation_matrix([x, y, z]),
                ts.euler_matrix(roll, pitch, yaw)
                )

                # ── camera→base_link (from your URDF, fixed offset) ─────────
                # adjust these to match where camera sits on the robot
                cam_x, cam_y, cam_z, cam_r, cam_p, cam_yaw = 0, 0, 0, 0, 0, 0   # metres forward, left, up
                T_cam_to_base = ts.concatenate_matrices(
                    ts.translation_matrix([-cam_x, -cam_y, -cam_z]),
                    ts.euler_matrix(cam_r, cam_p, cam_yaw)
                )

                # ── full chain: map → base_link ──────────────────────────────
                T_map_to_base = ts.concatenate_matrices(
                    T_map_to_tag,
                    T_tag_to_cam,
                    T_cam_to_base
                )

                # ── broadcast TF: map → base_link ───────────────────────────
                self._broadcast_tf(
                    T_map_to_base,
                    'map',
                    'base_link'
                )

                # ── publish as PoseWithCovarianceStamped for RTAB-Map ────────
                self._publish_pose(T_map_to_base)

                tmb = ts.translation_from_matrix(T_map_to_base)
                self.get_logger().info(
                    f'{tag_id}: map->base = {tmb[0]}, {tmb[1]}, {tmb[2]}',
                    throttle_duration_sec=0.5
                )

            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(
                    f'Could not use {tag_id}: {e}',
                    throttle_duration_sec=1.0
                )

    def _broadcast_tf(self, T, parent, child):
        t = ts.translation_from_matrix(T)
        q = ts.quaternion_from_matrix(T)

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x = float(t[0])
        msg.transform.translation.y = float(t[1])
        msg.transform.translation.z = float(t[2])
        msg.transform.rotation.x = float(q[0])
        msg.transform.rotation.y = float(q[1])
        msg.transform.rotation.z = float(q[2])
        msg.transform.rotation.w = float(q[3])

        self.broadcaster.sendTransform(msg)

    def _publish_pose(self, T):
        t = ts.translation_from_matrix(T)
        q = ts.quaternion_from_matrix(T)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = t[0]
        msg.pose.pose.position.y = t[1]
        msg.pose.pose.position.z = t[2]

        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        # 6x6 covariance matrix (x,y,z,rx,ry,rz)
        cov = [0.0] * 36
        cov[0] = self.position_variance     #x
        cov[7] = self.position_variance     #y
        cov[14] = 0.1                       #z - less important for ground robot
        cov[21] = self.orientation_variance #rx
        cov[28] = self.orientation_variance #ry
        cov[35] = self.orientation_variance #rz
        msg.pose.covariance = cov

        self.pose_pub.publish(msg)


            


def main(args=None):
    rclpy.init(args=args)
    april_sub = AprilTagLocalizationNode()
    rclpy.spin(april_sub)
    april_sub.destroy_node()
    rclpy.shutdown()
