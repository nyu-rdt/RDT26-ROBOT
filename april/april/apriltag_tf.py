import rclpy
from rclpy.node import Node
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
        self.tag_camera_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.tag_camera_tf)

        self.arena_tag_tf = TransformBroadcaster(self)
        self.arena_timer = self.create_timer(0.1, self.get_arena_tag_tf)

    def get_arena_tag_tf(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'arena'
        tf.child_frame_id = 'tag_19'
        #temporary numbers
        tf.transform.translation.x = 1.0
        tf.transform.translation.y = 1.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0

        self.arena_tag_tf.sendTransform(tf)


    def tag_camera_tf(self):
        try: 
            camera_tag = self.tf_buffer.lookup_transform('camera',
                                                        'tag_19',
                                                        rclpy.time.Time()
                                                        )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        translation = [camera_tag.transform.translation.x, 
                        camera_tag.transform.translation.y, 
                        camera_tag.transform.translation.z]
        
        rotation = [camera_tag.transform.rotation.x,
                    camera_tag.transform.rotation.y,
                    camera_tag.transform.rotation.z,
                    camera_tag.transform.rotation.w]
        
        camera_tag_tf = ts.concatenate_matrices(ts.translation_matrix(translation), 
                                                ts.quaternion_matrix(rotation))
        #invert matrix so that tf works around stationary tag to moving camera instead of static camera to moving tag
        tag_camera_tf = ts.inverse_matrix(camera_tag_tf)

        inverted_translation = ts.translation_from_matrix(tag_camera_tf)
        inverted_rotation = ts.quaternion_from_matrix(tag_camera_tf)

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'tag_19'
        tf.child_frame_id = 'camera'
        tf.transform.translation.x = inverted_translation[0]
        tf.transform.translation.y = inverted_translation[1]
        tf.transform.translation.z = inverted_translation[2]
        tf.transform.rotation.x = inverted_rotation[0]
        tf.transform.rotation.y = inverted_rotation[1]
        tf.transform.rotation.z = inverted_rotation[2]
        tf.transform.rotation.w = inverted_rotation[3]
    
        self.tag_camera_broadcaster.sendTransform(tf)


            


def main(args=None):
    rclpy.init(args=args)
    april_sub = AprilTagTF()
    rclpy.spin(april_sub)
    april_sub.destroy_node()
    rclpy.shutdown()
