"""
just seems like a dead file, i'd remove it
"""

import cv2
from cv_bridge import CvBridge
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

camera = cv2.VideoCapture(0)

while(True):
    ret, frame = camera.read(0)
    cv2.imshow("frame", frame)
        



# After the loop release the cap object 
camera.release() 
# Destroy all the windows 
cv2.destroyAllWindows() 