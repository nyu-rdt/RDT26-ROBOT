import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from pupil_apriltags import Detector
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster
import tf_transformations as ts

# Known tag poses in MAP frame 
# (x, y, z, roll, pitch, yaw) in metres/radians
TAG_POSES_IN_MAP = {
    19: (0.0, 0.0, 0.0, 0.0, 1.57, 0.0),   # starting zone tag 
}

# Calibration
fx, fy = 245.13135523728707, 268.4239205880632
cx, cy = 189.39414643843767, 150.29951972007822
camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
dist_coeffs   = np.array([0.7050221560988748, -3.7284367615305674, 0.013911897238276847, 0.001701746020378662, 5.078060359070571])

tag_sizes = {18: 0.069, 19: 0.245, 20: 0.196, 21: 0.069}

def world_tag_corners(tag_size):
    s = tag_size / 2
    return np.float32([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]])


class AprilTagLocalizationNode(Node):
    def __init__(self):
        super().__init__('apriltag_localization')

        self.broadcaster = TransformBroadcaster(self)
        self.pose_pub    = self.create_publisher(
            PoseWithCovarianceStamped, '/apriltag/robot_pose', 10
        )

        self.detector = Detector(
            families='tag36h11', nthreads=2,
            quad_decimate=1.5, refine_edges=True, decode_sharpening=0.25
        )

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 352)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 288)

        # Covariance to assign to apriltag-derived pose corrections
        # Tune these — smaller = more trust in tag, less in odometry
        self.position_variance    = 0.02   # metres^2
        self.orientation_variance = 0.01   # radians^2

        self.create_timer(0.05, self._process_frame)  # 20Hz

    def _process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        gray       = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)

        for det in detections:
            tag_id = det.tag_id
            if tag_id not in tag_sizes or tag_id not in TAG_POSES_IN_MAP:
                continue

            tag_size      = tag_sizes[tag_id]
            object_points = world_tag_corners(tag_size)
            image_points  = np.float32(det.corners)

            retval, rvec, tvec = cv2.solvePnP(
                object_points, image_points, camera_matrix, dist_coeffs
            )
            if not retval:
                continue

            # ── camera→tag transform from solvePnP ──────────────────────
            R, _ = cv2.Rodrigues(rvec)
            T_cam_to_tag = np.eye(4)
            T_cam_to_tag[:3, :3] = R
            T_cam_to_tag[:3, 3]  = tvec.ravel()

            # ── invert → tag→camera ──────────────────────────────────────
            T_tag_to_cam = ts.inverse_matrix(T_cam_to_tag)

            # ── map→tag (from known arena layout) ───────────────────────
            x, y, z, roll, pitch, yaw = TAG_POSES_IN_MAP[tag_id]
            T_map_to_tag = ts.concatenate_matrices(
                ts.translation_matrix([x, y, z]),
                ts.euler_matrix(roll, pitch, yaw)
            )

            # ── camera→base_link (from your URDF, fixed offset) ─────────
            # adjust these to match where camera sits on the robot
            cam_x, cam_y, cam_z = 0.0, 0.0, 0.0   # metres forward, left, up
            T_cam_to_base = ts.concatenate_matrices(
                ts.translation_matrix([-cam_x, -cam_y, -cam_z]),
                ts.euler_matrix(0, 0, 0)
            )

            # ── full chain: map → base_link ──────────────────────────────
            T_map_to_base = ts.concatenate_matrices(
                T_map_to_tag,
                T_tag_to_cam,
                T_cam_to_base
            )

            # ── broadcast TF: map → base_link ───────────────────────────
            self._broadcast_tf(T_map_to_base, 'map', 'base_link_apriltag', tag_id)

            # ── publish as PoseWithCovarianceStamped for RTAB-Map ────────
            self._publish_pose(T_map_to_base)

    def _broadcast_tf(self, T, parent, child, tag_id):
        t   = ts.translation_from_matrix(T)
        q   = ts.quaternion_from_matrix(T)

        tf  = TransformStamped()
        tf.header.stamp    = self.get_clock().now().to_msg()
        tf.header.frame_id = parent
        tf.child_frame_id  = f"{child}_{tag_id}"
        tf.transform.translation.x = t[0]
        tf.transform.translation.y = t[1]
        tf.transform.translation.z = t[2]
        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]
        self.broadcaster.sendTransform(tf)

    def _publish_pose(self, T):
        t = ts.translation_from_matrix(T)
        q = ts.quaternion_from_matrix(T)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
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
        cov[0]  = self.position_variance     # x
        cov[7]  = self.position_variance     # y
        cov[14] = 0.1                        # z — less important for ground robot
        cov[21] = self.orientation_variance  # rx
        cov[28] = self.orientation_variance  # ry
        cov[35] = self.orientation_variance  # rz
        msg.pose.covariance = cov

        self.pose_pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()