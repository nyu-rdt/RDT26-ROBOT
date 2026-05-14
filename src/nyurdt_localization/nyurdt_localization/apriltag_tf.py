'''
apriltag_tf.py
'''
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import tf_transformations as ts
from apriltag_msgs.msg import AprilTagDetectionArray

# Known tag poses in MAP frame.
# Origin: x=0 at fiducial wall, y=0 at arena floor — matches autonomy_manager.py zone system.
# Valid for both left and right arenas: a left-right mirror does not change tag y positions.
# Fiducial rail centre: x=0.0, y=3.47 (midpoint of starting zone y:(2.47, 4.47)).
# Tags span y=3.026 to y=3.914, symmetric around y=3.47.
# tag_0 is at the bottom of the rail, tag_21 at the top.
# (x, y, z, roll, pitch, yaw) in metres/radians
TAG_POSES_IN_MAP = {
    'tag_0':  (0.0, 3.0259, 0.4300, 0.0, 1.57, 1.57),
    'tag_1':  (0.0, 3.2702, 0.5316, 0.0, 1.57, 1.57),
    'tag_2':  (0.0, 3.2702, 0.3729, 0.0, 1.57, 1.57),
    'tag_18': (0.0, 3.4700, 0.4300, 0.0, 1.57, 1.57),
    'tag_19': (0.0, 3.6698, 0.5316, 0.0, 1.57, 1.57),
    'tag_20': (0.0, 3.6698, 0.3729, 0.0, 1.57, 1.57),
    'tag_21': (0.0, 3.9141, 0.4300, 0.0, 1.57, 1.57),
}

class AprilTagLocalizationNode(Node):
    def __init__(self):
        super().__init__('apriltag_localization')

        # Create instance of buffer and listener to subscribe to /tf topic and store transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.front_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/front_cam/detections',
            self._process_front,
            10
        )

        self.left_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/left_cam/detections',
            self._process_left,
            10
        )

        # self.right_sub = self.create_subscription(
        #     AprilTagDetectionArray,
        #     '/right_cam/detections',
        #     self._process_right,
        #     10
        # )
    
        self.back_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/back_cam/detections',
            self._process_back,
            10
        )

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/apriltag/robot_pose',
            10
        )

        # Covariance to assign to apriltag-derived pose corrections
        # Tune these — smaller = more trust in tag, less in odometry
        self.position_variance = 0.02 # metres^2
        self.orientation_variance = 0.01 # radians^2
    def _process_front(self, msg):
        self._process_frames(msg, 'front_camera_optical_frame_link')

    def _process_left(self, msg):
        self._process_frames(msg, 'left_camera_optical_frame_link')

    # def _process_right(self, msg):
    #     self._process_frames(msg, 'right_camera_optical_frame_link')

    def _process_back(self, msg):
        self._process_frames(msg, 'back_camera_optical_frame_link')

    def _process_frames(self, msg, camera_frame):
        if not msg.detections:
            return
        
        # Variables to determine average pose from multiple tags later
        sum_x = sum_y = sum_z = 0.0
        sum_sin_yaw = sum_cos_yaw = 0.0
        sum_w = 0.0
        

        for det in msg.detections:
            tag_id = f'tag_{det.id}'
            w = det.decision_margin

            if tag_id not in TAG_POSES_IN_MAP or w < 50: # decision margin determines confidence in detection
                continue

            try:
                # ── tag -> camera transform
                tag_to_cam_tf = self.tf_buffer.lookup_transform(
                    camera_frame,
                    tag_id,
                    Time()
                )

                tc_t = tag_to_cam_tf.transform.translation
                tc_q = tag_to_cam_tf.transform.rotation
                self.get_logger().info(f"{tag_id}-> {camera_frame} dist =  {self.distance(tc_t.x, tc_t.y, tc_t.z)}m", throttle_duration_sec=0.5)

                tc_trans = ts.translation_matrix([tc_t.x, tc_t.y, tc_t.z])
                tc_rot = ts.quaternion_matrix([tc_q.x, tc_q.y, tc_q.z, tc_q.w])

                T_tag_to_cam = ts.concatenate_matrices(tc_trans, tc_rot)

                # ── map→tag (from known arena layout) ───────────────────────
                x, y, z, roll, pitch, yaw = TAG_POSES_IN_MAP[tag_id]
                T_map_to_tag = ts.concatenate_matrices(
                ts.translation_matrix([x, y, z]),
                ts.euler_matrix(roll, pitch, yaw)
                )

                # ── camera→base_link (from robot.xacro) ─────────

                cam_to_base_tf = self.tf_buffer.lookup_transform(
                    'base_link',
                    camera_frame,
                    Time()
                )

                cb_t = cam_to_base_tf.transform.translation
                cb_q = cam_to_base_tf.transform.rotation

                cb_trans = ts.translation_matrix([cb_t.x, cb_t.y, cb_t.z])
                cb_rot = ts.quaternion_matrix([cb_q.x, cb_q.y, cb_q.z, cb_q.w])

                T_cam_to_base = ts.concatenate_matrices(cb_trans, cb_rot)


                # ── full chain: map → base_link ──────────────────────────────
                T_map_to_base = ts.concatenate_matrices(
                    T_map_to_tag,
                    T_tag_to_cam,
                    T_cam_to_base
                )

                tmb = ts.translation_from_matrix(T_map_to_base)
                self.get_logger().info(f'{tag_id}: map->base dist = {self.distance(tmb[0], tmb[1], tmb[2])}m', throttle_duration_sec=0.5)

                qymb = ts.euler_from_matrix(T_map_to_base)[2] # using only yaw for averaging due to robot ground rotation

                sum_x += tmb[0] * w
                sum_y += tmb[1] * w
                sum_z += tmb[2] * w
                sum_sin_yaw += math.sin(qymb) * w
                sum_cos_yaw += math.cos(qymb) * w
                sum_w += w

            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(f'Could not use {tag_id}: {e}', throttle_duration_sec=1.0)

        # Compute weighted average pose based on decision margin
        if sum_w == 0:
            return
        avg_x = sum_x / sum_w
        avg_y = sum_y / sum_w
        avg_z = sum_z / sum_w
        avg_yaw = math.atan2(sum_sin_yaw / sum_w, sum_cos_yaw / sum_w)
        Tavg_map_to_base = ts.concatenate_matrices(
            ts.translation_matrix([avg_x, avg_y, avg_z]),
            ts.euler_matrix(0, 0, avg_yaw)
        )
        
        # ── publish as PoseWithCovarianceStamped for relocalization tag check ────────
        self._publish_pose(Tavg_map_to_base)
        avg_t = ts.translation_from_matrix(Tavg_map_to_base)
        self.get_logger().info(f'averaged map->base dist = {self.distance(avg_t[0], avg_t[1], avg_t[2])}m', throttle_duration_sec=0.5)

    def distance(self, x, y, z):
        return math.sqrt(x**2 + y**2 + z**2)

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
