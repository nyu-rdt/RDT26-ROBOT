import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import tf_transformations
import tf2_py
import cv2
from time import sleep
import time
import numpy as np
from camera_pkg.tag_information import TAG_OBJECT_POINTS
from scipy.spatial.transform import Rotation as R
import apriltag


" This node will pulish apriltag transforms giving the position and orientation of"
"the apriltag relative to the camera"

class AprilTagPosePublisher(Node):

    def __init__(self):
        super().__init__('transform_publisher_node')
        # self.bridge = CvBridge()  # Create a CvBridge object for image conversion
        self.broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Image,
            # 'april_tag',
            '/webcam_front/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info("Camera subscriber node started...")

        # NEW ARRAY CALIBRATED ON 4/5/25 BY MICHAEL
        # CAMERA WITH PURPLE STICKER
        self.K = np.array([
            [655.47, 0.0, 324.70],
            [0.0, 657.41, 272.43],
            [0.0, 0.0, 1.0]])
        
        # CALBRIATED 4/6/25 BY MICHAEL
        # CAMERA WITH WHITE BASE
        # self.K = np.array([
        #     [723.816315, 0.000000, 272.337177],
        #     [0.000000, 723.940470, 208.623707],
        #     [0.000000, 0.000000, 1.000000]])
        
        # CAMERA WITH PURPLE STICKER DISTORTION COEFFS
        self.dist_coeffs = np.array([-0.050, -0.0024, -0.0022, -0.0005, 0])

        # CAMERA WITH WHITE BASE
        # self.dist_coeffs = np.array([-0.055869, -0.057321, 0.005996, 0.000533, 0.000000])

        self.options = apriltag.DetectorOptions(families='tag36h11')
        self.detector = apriltag.Detector(self.options)
        
    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to OpenCV format
            # frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            image_data = np.frombuffer(msg.data, dtype=np.uint8)  # Convert byte data to numpy array
            frame = image_data.reshape((msg.height, msg.width, 3))
        
            # Display the frame using OpenCV
            cv2.imshow('Received Camera Frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                rclpy.shutdown()  # Shut down ROS after closing the window
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

        pose_matrix = self.get_pose(frame)

        if pose_matrix is not None:
            self.publish_transform(pose_matrix)
        else:
            self.get_logger().warn("No pose found for current frame")


    def get_pose(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results_raw  = self.detector.detect(gray)

        results = [tag for tag in results_raw if tag.tag_id in TAG_OBJECT_POINTS]
        
        if not results:
            return None
        
        berm_position_one_obj_points = []
        berm_position_one_all_corners = []
    
        # filling up each respective array with points
        for tag in results:
            tag_points = TAG_OBJECT_POINTS[tag.tag_id]
            corners = np.array(tag.corners).astype(np.float32)

            # specify which board is received
            berm_position_one_obj_points.extend(tag_points)
            berm_position_one_all_corners.extend(corners)

            print(tag.tag_id)

        # convert to numpy array
        berm_position_one_obj_points = np.asarray(berm_position_one_obj_points)
        berm_position_one_all_corners = np.asarray(berm_position_one_all_corners)
        
        ret, rvec, tvec = cv2.solvePnP(berm_position_one_obj_points, berm_position_one_all_corners, self.K, self.dist_coeffs)

        rmat, _ = cv2.Rodrigues(rvec)

        self.draw_results(results, frame)
        # have cv2 draw out the borders for the april tags

        print(f"x distance: {tvec.flatten()[0]}, y distance: {tvec.flatten()[1]}, z distance: {tvec.flatten()[2]}")
        print(f"x angle: {rvec.flatten()[0]}, y angle: {rvec.flatten()[1]}, z angle: {rvec.flatten()[2]}")
        print(rmat)
        print(tvec)

        try:
            T = np.eye(4)
            print(T)
            T[:3, :3] = rmat
            T[:3, 3] = tvec.flatten()
            print("Matrix after assignment:")
            print(T)
        except Exception as e:
            print(f"Error assigning to T: {e}")


        # T[:3,:3] = rmat
        # T[:3, 3] = tvec.flatten()

        self.get_logger().info(f"Transformation Matrix:\n{np.array2string(T, precision=3)}")

        return T
    
    def draw_results(self, results, image):
        for r in results:
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))

            cv2.line(image, ptA, ptB, (0, 255, 0), 2)
            cv2.line(image, ptB, ptC, (0, 255, 0), 2)
            cv2.line(image, ptC, ptD, (0, 255, 0), 2)
            cv2.line(image, ptD, ptA, (0, 255, 0), 2)

            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)

            tagFamily = r.tag_family.decode("utf-8")
                
        cv2.imshow("AprilTag Detections", image)

        cv2.waitKey(1)
        # time.sleep(0.1)
        
    def publish_transform(self, matrix):
        translation = matrix[0:3, 3]
        rotation_matrix = matrix[:3, :3]
        r = R.from_matrix(rotation_matrix)  # Create a Rotation object
        quaternion = r.as_quat()

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_link' # base_link
        t.child_frame_id = 'apriltag_object' # detected_object

        t.transform.translation.x = float(translation[0])
        t.transform.translation.y = float(translation[1])
        t.transform.translation.z = float(translation[2])
        t.transform.rotation.x = float(quaternion[0])
        t.transform.rotation.y = float(quaternion[1])
        t.transform.rotation.z = float(quaternion[2])
        t.transform.rotation.w = float(quaternion[3])
        
        self.broadcaster.sendTransform(t)
        self.get_logger().info(f"Published transform at {translation}")


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
    
if __name__ == "__main__":
    main()