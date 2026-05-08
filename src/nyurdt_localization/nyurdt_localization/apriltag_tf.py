'''
apriltag_tf.py
'''
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, LookupException, ConnectivityException, ExtrapolationException
import tf_transformations as ts
from apriltag_msgs.msg import AprilTagDetectionArray

# Known tag poses in MAP frame.
# Coordinate system: X=0 (construction/left wall) → X=8.10 (fiducial/right wall).
#   Y: −2.275 → +2.275 m    Z: 0 = ground
# Format: (x, y, z, roll, pitch, yaw)  [metres / radians]
# Tags are on the RIGHT wall (x=8.10), face normal toward −X (into the arena).
# Orientation: roll=π/2, pitch=0, yaw=−π/2  →  tag Z-axis = −X world axis.
_R = math.pi / 2
# Map origin = centre of 2×2m starting zone (SDF world 7.10, 1.28).
# All positions = SDF world position − (7.10, 1.28).
# Tags on right wall: SDF x=8.10 → map x=1.00 (1m right of map origin).
TAG_POSES_IN_MAP = {
    # Fiducial rail on right wall, y values in map frame (SDF y − 1.28)
    'tag_0':  (1.00, 0.056, 0.43,   _R, 0.0, -_R),
    'tag_1':  (1.00, 0.300, 0.5316, _R, 0.0, -_R),
    'tag_2':  (1.00, 0.300, 0.3729, _R, 0.0, -_R),
    'tag_18': (1.00, 0.500, 0.43,   _R, 0.0, -_R),
    'tag_19': (1.00, 0.700, 0.5316, _R, 0.0, -_R),
    'tag_20': (1.00, 0.700, 0.3729, _R, 0.0, -_R),
    'tag_21': (1.00, 0.944, 0.43,   _R, 0.0, -_R),
}

#bottom right starting zone
# TAG_POSES_IN_MAP = {
#     'tag_0': (-1.0, 0.4441, .43, 0.0, 1.57, 1.57),
#     'tag_1': (-1.0, 0.1998, .5316, 0.0, 1.57, 1.57),
#     'tag_2': (-1.0, 0.1998, 0.3729, 0.0, 1.57, 1.57),
#     'tag_18': (-1.0, 0, .43,0, 1.57, 1.57),
#     'tag_19': (-1.0, -0.1998, .5316, 0.0, 1.57, 1.57),
#     'tag_20': (-1.0, -0.1998, .3729, 0.0, 1.57, 1.57),
#     'tag_21': (-1.0, -0.4441, .43, 0.0, 1.57, 1.57),
#     # 'tag_0': (0, 0,0, 0.0, 1.57, 0.0),
#     # 'tag_1': (0, 0, 0.0, 0.0, 1.57, 0.0),
#     # 'tag_2': (0, 0, 0.0, 0.0, 1.57, 0.0),
#     # 'tag_18': (0, 0.0, 0.0, 0.0, 1.57, 0.0),
#     # 'tag_19': (0, 0.0,0, 0.0, 1.57, 0.0),
#     # 'tag_20': (0, 0.0,0.0, 0.0, 1.57, 0.0),
#     # 'tag_21': (0, 0.0,0.0,0.0, 1.57, 0.0),
# }

class AprilTagLocalizationNode(Node):
    def __init__(self):
        super().__init__('apriltag_localization')

        # Create instance of buffer and listener to subscribe to /tf topic and store transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)

        # Real robot: physical USB cameras on all 4 sides
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

        # Sim: RealSense detections on /sim_cam/detections → _process_sim uses camera_link.
        # apriltag_back_rtabmap independently feeds /rtabmap/apriltag for RTAB-Map
        # landmarks — they must NOT share this subscription or back-cam detections
        # get computed through the wrong camera frame (RealSense vs back cam).
        self.sim_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/sim_cam/detections',
            self._process_sim,
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

    def _process_sim(self, msg):
        # camera_link is the RealSense TF root; camera_frame_alias bridges it to
        # the Gazebo sensor frame (rdt26_robot/base_link/realsense_d435) so the
        # TF lookup tag→camera_link→base_link resolves correctly in sim.
        self._process_frames(msg, 'camera_link')

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
        
        # ── publish as PoseWithCovarianceStamped ─────────────────────
        # For simulation purposes:
        # Broadcasting a direct map→base_link TF conflicts with the proper chain
        # map→odom→base_link (from Gazebo/wheel-odom), causing RViz to flash as
        # TF alternates between the two paths.
        # RTAB-Map receives the tag detections via /rtabmap/apriltag (landmarks).
        # autonomy_manager reads /apriltag/robot_pose only for the timestamp.
        self._publish_pose(Tavg_map_to_base)
        avg_t = ts.translation_from_matrix(Tavg_map_to_base)
        self.get_logger().info(f'averaged map->base dist = {self.distance(avg_t[0], avg_t[1], avg_t[2])}m', throttle_duration_sec=0.5)

    def distance(self, x, y, z):
        return math.sqrt(x**2 + y**2 + z**2)

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
