import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import numpy as np
#gad26 this works

"This Node takes in the camera feed then publishes it to the apriltag topic."
class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.vid = cv2.VideoCapture(2)
         # Check if the camera is opened correctly
        if not self.vid.isOpened():
            raise RuntimeError("Failed to open camera.")
            # self.get_logger().error("Failed to open camera.")
            # rclpy.shutdown()  # Shutdown ROS if camera cannot be opened
            # return
        
        self.publisher = self.create_publisher(Image, 'april_tag', 10)  # Create a publisher for the 'april_tag' topic
        self.timer = self.create_timer(1/30.0, self.main_control)  # 30 FPS

    def opencv_to_ros_image(self, frame):
        ros_image = Image()
        ros_image.header = Header()  # Set the header fields
        ros_image.header.stamp = rclpy.time.Time().to_msg()
        ros_image.height = frame.shape[0]
        ros_image.width = frame.shape[1]
        ros_image.encoding = "bgr8"  # Or "mono8" if grayscale
        ros_image.is_bigendian = False
        ros_image.step = frame.shape[1] * 3  # 3 channels for bgr8
        ros_image.data = np.asarray(frame).tobytes()  # Convert numpy array to bytes
        return ros_image
    

    def main_control(self):
        ret, frame = self.vid.read()
        if not ret:
            self.get_logger().warn("Failed to grab frame.")
            return
        
        # Convert OpenCV image to ROS Image message
        try:
            ros_image = self.opencv_to_ros_image(frame)
            self.publisher.publish(ros_image)  # Publish the frame
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

        cv2.imshow('Camera Frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.vid.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()  # Shut down ROS after releasing the camera

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("CameraNode interrupted by user.")
    except Exception as e:
        print(f"CameraNode failed to start or crashed: {e}")
    finally:
        if node and hasattr(node, 'vid'):
            node.vid.release()
        cv2.destroyAllWindows()
        rclpy.shutdown()