import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
# from tf_transformations import quaternion_matrix
from scipy.spatial.transform import Rotation as R
import numpy as np

class AprilTagTransformListener(Node):

    def __init__(self):
        super().__init__('apriltag_transform_listener')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.5, self.timer_callback)  # Check every 0.5s

    def timer_callback(self):
        try:
            # now = rclpy.time.Time()
            # now = self.get_clock().now().to_msg()
            trans = self.tf_buffer.lookup_transform(
                'camera_link',
                'apriltag_object',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=5.0)
            )

            translation = trans.transform.translation
            rotation = trans.transform.rotation

            # Convert quaternion to 3x3 rotation matrix
            rot = R.from_quat([
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w
            ])
            rmat = rot.as_matrix()

            # Build 4x4 homogeneous transformation matrix
            T = np.eye(4)
            T[:3, :3] = rmat
            T[:3, 3] = [translation.x, translation.y, translation.z]
            
            # # Rebuild the transformation matrix (4x4)
            # T = quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
            # T[0, 3] = translation.x
            # T[1, 3] = translation.y
            # T[2, 3] = translation.z

            self.get_logger().info(f"Transformation Matrix:\n{np.array2string(T, precision=3)}")

        except Exception as e:
            self.get_logger().warn(f"Transform not available: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagTransformListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
