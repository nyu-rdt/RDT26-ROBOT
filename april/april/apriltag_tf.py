import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
from apriltag_msgs.msg import AprilTagDetectionArray  

class AprilTagTF(Node):
    def __init__(self):
        super().__init__('april_tf')
        # self.subscription = self.create_subscription(
        #     AprilTagDetectionArray,
        #     '/detections',
        #     self.listener_callback,
        #     10)
        self.buffer = Buffer()

        

        #camera to tag
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tag_tf_listener,
            10)
        

    def tag_tf_listener(self, msg):
        # for detection in msg.detections:
        #     tag_id = detection.id[0]
        #     detected_pose = detection.pose.pose.pose
        #     self.get_logger().info(f'AprilTag ID: {tag_id}')
        #     self.get_logger().info(f'Pose: {detected_pose}')
        self.get_logger().info('Receiving TFMessage...')
        for transform in msg.transforms:
            tag_to_camera = self.buffer.lookup_transform(transform.header.frame_id, 
                                         transform.child_frame_id, 
                                         rclpy.time.Time())
            self.get_logger().info(f'Transform from {transform.header.frame_id} to {transform.child_frame_id}: \n {tag_to_camera.transform}')


def main(args=None):
    rclpy.init(args=args)
    april_sub = AprilTagTF()
    rclpy.spin(april_sub)
    april_sub.destroy_node()
    rclpy.shutdown()
   

