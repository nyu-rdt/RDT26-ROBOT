'''
Detects AprilTag boards created for the 2024-2025 season
Calculates the linear distances (x,y,z) between the tags and the cameras
'''

from tag_information import TAG_OBJECT_POINTS
from time import sleep
import time

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import apriltag

# create a subscriber node which subscribes to the topic /robotData/sensors
class AprilTagDetector:

    def __init__(self):

        # Video capture
        self.cap = cv2.VideoCapture("/dev/video0")
        print(self.cap)

        # self.K = np.array([
        #     [631.42, 0.0, 325.59],
        #     [0.0, 631.408, 231.969],
        #     [0.0, 0.0, 1.0]])

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
                
        cv2.imshow("Image", image)

        cv2.waitKey(1)
        time.sleep(0.1)
        
    def get_pose(self):
        ret, image = self.cap.read()
        if not ret:
            return False, None, None
            
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        results_raw  = self.detector.detect(gray)

        results = [tag for tag in results_raw if tag.tag_id in TAG_OBJECT_POINTS]
        
        if not results:
            return False, None, None
        
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
        euler = R.from_matrix(rmat).as_euler('xyz', degrees=True)
        
        # imgpts, _ = cv2.projectPoints(berm_position_one_obj_points, rvec, tvec, self.K, self.dist_coeffs)
        # for pt in imgpts:
        #     cv2.circle(image, tuple(pt.ravel().astype(int)), 5, (0, 0, 255), -1)

        self.draw_results(results, image)
        # have cv2 draw out the borders for the april tags

        print(f"x distance: {tvec.flatten()[0]}, y distance: {tvec.flatten()[1]}, z distance: {tvec.flatten()[2]}")
        print(f"x angle: {rvec.flatten()[0]}, y angle: {rvec.flatten()[1]}, z angle: {rvec.flatten()[2]}")

        T = np.eye(4)

        T[:3,:3] = rmat

        T[:3, 3] = tvec.flatten()

        print(T)

        return True, tvec.flatten(), euler
    

def main():

    depo_node = AprilTagDetector()

    while True:
        depo_node.get_pose()
        time.sleep(0.1)

if __name__ == "__main__":
    main()
