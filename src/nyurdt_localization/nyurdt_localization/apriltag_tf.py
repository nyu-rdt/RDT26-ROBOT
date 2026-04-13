'''
apriltag_tf.py
'''
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, LookupException, ConnectivityException, ExtrapolationException
import tf_transformations as ts

class AprilTagTF(Node):
    def __init__(self):
        super().__init__('april_tf')
        #create instance of transform broadcasters to publish tf from arena to tag and tag to camera
        #as well as a buffer and listener to subscribe to /tf topic and store transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)
        #self.tag_camera_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.tag_camera_tf)



    def tag_camera_tf(self):
        frames = ['tag_18', 'tag_19', 'tag_20', 'tag_21']
        for frame in frames: 
            try: 
                camera_tag = self.tf_buffer.lookup_transform('camera_optical_frame_link',
                                                            frame,
                                                            Time(),
                                                            )
                self.get_logger().info(f"Horizontal Distance: {camera_tag.transform.translation.z}", throttle_duration_sec=.5)
            except Exception as e:
                self.get_logger().warn(f"{e}")            
                return
        

#CODE BELOW IS ONLY NECESSARY WHEN TRYING TO MODIFY TRANSFORMS

    #     translation = [camera_tag.transform.translation.x, 
    #                     camera_tag.transform.translation.y, 
    #                     camera_tag.transform.translation.z]
        
    #     rotation = [camera_tag.transform.rotation.x,
    #                 camera_tag.transform.rotation.y,
    #                 camera_tag.transform.rotation.z,
    #                 camera_tag.transform.rotation.w]
        
        # camera_tag_tf = ts.concatenate_matrices(ts.translation_matrix(translation), 
        #                                         ts.quaternion_matrix(rotation))
        #invert matrix so that tf works around stationary tag to moving camera instead of static camera to moving tag
        # tag_camera_tf = ts.inverse_matrix(camera_tag_tf)

        # inverted_translation = ts.translation_from_matrix(tag_camera_tf)
        # inverted_rotation = ts.quaternion_from_matrix(tag_camera_tf)

        # tf = TransformStamped()
        # tf.header.stamp = self.get_clock().now().to_msg()
        # tf.header.frame_id = 'tag_19'
        # tf.child_frame_id = 'camera_optical_frame_link'
        # tf.transform.translation.x = inverted_translation[0]
        # tf.transform.translation.y = inverted_translation[1]
        # tf.transform.translation.z = inverted_translation[2]
        # tf.transform.rotation.x = inverted_rotation[0]
        # tf.transform.rotation.y = inverted_rotation[1]
        # tf.transform.rotation.z = inverted_rotation[2]
        # tf.transform.rotation.w = inverted_rotation[3]

        # self.tag_camera_broadcaster.sendTransform(tf)


            


def main(args=None):
    rclpy.init(args=args)
    april_sub = AprilTagTF()
    rclpy.spin(april_sub)
    april_sub.destroy_node()
    rclpy.shutdown()
