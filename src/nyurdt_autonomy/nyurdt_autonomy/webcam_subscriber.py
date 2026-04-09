"""
This file is named webcam_subscriber.py but actually opens the webcam and publishes

This file is also never spun up anywhere?
"""

import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import numpy as np

"This Node takes in the camera feed then publishes it to the apriltag topic."
class WebcamNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        # You might wanna make this a ros parameter, since this can number may (or may not) change
        self.vid = cv2.VideoCapture(2)
         # Check if the camera is opened correctly
        if not self.vid.isOpened():
            raise RuntimeError("Failed to open camera.")
        
            # Remove comments
            # self.get_logger().error("Failed to open camera.")
            # rclpy.shutdown()  # Shutdown ROS if camera cannot be opened
            # return
        
        # Usually you also want to publish the CameraInfo for your webcam
        self.publisher = self.create_publisher(Image, 'april_tag', 10)  # Create a publisher for the 'april_tag' topic
        self.timer = self.create_timer(1/30.0, self.main_control)  # 30 FPS

    # Is there not an existing opencv to ros bridge?
    #   https://github.com/ros-perception/vision_opencv/tree/rolling/cv_bridge
    #   https://docs.ros.org/en/jazzy/Related-Projects/Intel-ROS2-Projects.html
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
        # You may not always get the most recent frame here
        #   I think there's an internal buffer that CV2 has? Need to double check
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

        # Is this intended for autonomy?
        #   I would put this cv2.imshow behind a flag or something
        cv2.imshow('Camera Frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.vid.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()  # Shut down ROS after releasing the camera

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WebcamNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("WebCamNode interrupted by user.")
    except Exception as e:
        print(f"WebcamNode failed to start or crashed: {e}")
    finally:
        if node and hasattr(node, 'vid'):
            node.vid.release()
        cv2.destroyAllWindows()
        rclpy.shutdown()