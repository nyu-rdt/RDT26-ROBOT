"""
This also just seems to be legacy code?
Not too obvious what the point of this file is, but probably just use whatever realsense nodes come provided
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import pyrealsense2 as rs2
import cv2

class RealSensePublisher(Node):
    def __init__(self):
        super().__init__('realsense_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, 'pointcloud', 10)
        self.pipeline = rs2.pipeline()
        self.config = rs2.config()
        self.config.enable_stream(rs2.stream.depth)
        self.pipeline.start(self.config)

    def create_point_cloud_msg(self, depth_frame):
        # Extracting depth data from the frame
        depth_image = np.asanyarray(depth_frame.get_data())
        # Assuming 640x480 resolution, adjust as per your camera
        height, width = depth_image.shape[:2]
        fx, fy = width / 2, height / 2
        cx, cy = width / 2, height / 2
        f = 0.5
        # Generating point cloud data
        points = []
        for y in range(0, height):
            for x in range(0, width):
                d = depth_image[y, x] / 1000.0  # convert mm to meters
                if d == 0:
                    continue
                z = d / f
                x = (x - cx) * z / fx
                y = (y - cy) * z / fy
                points.append([x, y, z])
        # Converting point cloud data to ROS PointCloud2 message
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'  # Adjust frame_id as per your setup
        msg.height = height
        msg.width = width
        msg.fields.append(PointField(
            name='x',
            offset=0,
            datatype=PointField.FLOAT32,
            count=1
        ))
        msg.fields.append(PointField(
            name='y',
            offset=4,
            datatype=PointField.FLOAT32,
            count=1
        ))
        msg.fields.append(PointField(
            name='z',
            offset=8,
            datatype=PointField.FLOAT32,
            count=1
        ))
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * width
        msg.is_dense = False
        msg.data = np.asarray(points, dtype=np.float32).tobytes()
        return msg

    def run(self):
        while rclpy.ok():
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue
            pointcloud_msg = self.create_point_cloud_msg(depth_frame)
            self.publisher_.publish(pointcloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RealSensePublisher()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


