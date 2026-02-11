import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
# import rclpy.time

from rplidar import RPLidar
import numpy as np
import math


class LidarPublisher(Node):

    LIDAR_EFFECTIVE_DISTANCE = 5
    LIDAR_HEIGHT = 0.38

    def __init__(self):
        super().__init__('LidarPublisher')


        # create publisher that publishes PointCloud2 message
        self.publisher = self.create_publisher(sensor_msgs.PointCloud2, '/transform', 10)

        # intialize a timer
        timer_period = 1 / 30.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    
    def timer_callback(self):
        # get data points from the lidar
        data = self._get_scan()

        # convert datapoints from polar to rectangular coordinates
        datapoints = self._process_data(data)

        # make the pointcloud 
        pointcloud = self._make_pointcloud(datapoints, 'map')

        # publish pointcloud
        self.publisher.publish(pointcloud)

        self.i += 1
        print(self.i, len(datapoints))

    def _get_scan(self):
        """
        Get scan data from the lidar

        return:
            data: [intensity, angle, distance]
        """

        data = []
        # initialize lidar
        lidar = RPLidar('/dev/ttyUSB0')

        # print('Recording measurments... Press Ctrl+C to stop.')
        try:
            data = np.array(next(lidar.iter_scans()))
        except KeyboardInterrupt:
            raise Exception('interrupted')
        except:
            pass
        finally:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        return data


    def _process_data(self, data):
        """
        process the lidar scan data, turn it from polar coordinates to rectangular coordinates

        input: 
            data = [(intensity, angle, distance)]
        return:
            [np.array[(x, y)]
        """
        datapoints = []
        z = LidarPublisher.LIDAR_HEIGHT
        for intensity, angle, distance in data:
            # print(f"intensity, angle, distance: {intensity, angle, distance}")
            # convert angle from degree to radians
            angle = (angle / 180) * math.pi

            # convert distance from mm to m
            distance = distance * 0.001

            # check if the measured distance is in valid range (5m)
            if distance > LidarPublisher.LIDAR_EFFECTIVE_DISTANCE:
                continue

            x = math.cos(angle) * distance
            y = math.sin(angle) * distance
            datapoints.append((x, y, z))
        return np.array(datapoints)
    

    def _make_pointcloud(self, points, parent_frame):
        """ Creates a point cloud message.
        Args:
            points: Nx3 array of xyz positions.
            parent_frame: frame in which the point cloud is defined
        Returns:
            sensor_msgs/PointCloud2 message

        Code source:
            https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0

        References:
            http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html
            http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointField.html
            http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html
        
        Credit:
            This method is diretly copied from:
            https://github.com/SebastianGrans/ROS2-Point-Cloud-Demo/blob/master/pcd_demo/pcd_publisher/pcd_publisher_node.py#L58C1-L104C6
        """
        # In a PointCloud2 message, the point cloud is stored as an byte 
        # array. In order to unpack it, we also include some parameters 
        # which desribes the size of each individual point.
        ros_dtype = sensor_msgs.PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

        data = points.astype(dtype).tobytes() 

        # The fields specify what the bytes represents. The first 4 bytes 
        # represents the x-coordinate, the next 4 the y-coordinate, etc.
        fields = [sensor_msgs.PointField(
            name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyz')]

        # The PointCloud2 message also has a header which specifies which 
        # coordinate frame it is represented in. 
        header = std_msgs.Header(frame_id=parent_frame)

        return sensor_msgs.PointCloud2(
            header=header,
            height=1, 
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 3), # Every point consists of three float32s.
            row_step=(itemsize * 3 * points.shape[0]),
            data=data
        )
    
def main(args=None):
    rclpy.init(args = args)
    LidarPublisher = LidarPublisher()
    rclpy.spin(LidarPublisher)

if __name__ == "__main__":
    main()
