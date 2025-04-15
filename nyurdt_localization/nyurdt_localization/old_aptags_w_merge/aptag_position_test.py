'''
Detects AprilTag boards created for the 2023-2024 season
Calculates the linear distances (x,y,z) between the tags and the cameras
'''

# from tag_information_new import TAG_OBJECT_POINTS

from tag_information_new_new import TAG_OBJECT_POINTS
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

        self.K = np.array([
            [631.42, 0.0, 325.59],
            [0.0, 631.408, 231.969],
            [0.0, 0.0, 1.0]])

        self.options = apriltag.DetectorOptions(families='tag36h11')
        self.detector = apriltag.Detector(self.options)

    # main april tag detection function for tags 7-12, 18-21

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

        # points and corners for board with 6 tags
        
        berm_position_one_obj_points = []
        berm_position_one_all_corners = []

        # points and corners for board with 4 tags
        berm_position_three_obj_points = []
        berm_position_three_all_corners = []
    
        # filling up each respective array with points
        for tag in results:
            tag_points = TAG_OBJECT_POINTS[tag.tag_id]
            corners = np.array(tag.corners).astype(np.float32)
            
            if (tag.tag_id >= 7 and tag.tag_id <= 12): 
                berm_position_one_obj_points.extend(tag_points)
                berm_position_one_all_corners.extend(corners)
            else: 
                berm_position_three_obj_points.extend(tag_points)
                berm_position_three_all_corners.extend(corners)

            print(tag.tag_id)


        # convert to numpy array
        # board with  tags: #7-12 (coolerest board)
        berm_position_one_obj_points = np.asarray(berm_position_one_obj_points)
        berm_position_one_all_corners = np.asarray(berm_position_one_all_corners)

        # board with tags #18-21 (uncoolestest board)
        berm_position_three_obj_points = np.asarray(berm_position_three_obj_points)
        berm_position_three_all_corners = np.asarray(berm_position_three_all_corners)

        #ret, rvec, tvec = cv2.solvePnP(berm_position_three_obj_points, berm_position_three_all_corners, self.K, None)

        # grab the rotation and translation vectors
        if berm_position_one_all_corners.size > 0:
            ret, rvec, tvec = cv2.solvePnP(berm_position_one_obj_points, berm_position_one_all_corners, self.K, None)

        elif berm_position_three_all_corners.size > 0:
            ret, rvec, tvec = cv2.solvePnP(berm_position_three_obj_points, berm_position_three_all_corners, self.K, None)

        rmat, _ = cv2.Rodrigues(rvec)
        euler = R.from_matrix(rmat).as_euler('xyz', degrees=True)


        # if array is not empty, get the rotation and translation vector for each respective board
        # take the average of two boards if both in view
        if berm_position_one_obj_points.size != 0 and berm_position_three_obj_points.size != 0:
            tvec = tvec.flatten()  
            a_ret, a_rvec, a_tvec = cv2.solvePnP(berm_position_one_obj_points, berm_position_one_all_corners, self.K, None)
            m_ret, m_rvec, m_tvec = cv2.solvePnP(berm_position_three_obj_points, berm_position_three_all_corners, self.K, None)
            tvec[0] = (a_tvec.flatten()[0] + m_tvec.flatten()[0]) / 2.0
            tvec[1] = (a_tvec.flatten()[1] + m_tvec.flatten()[1]) / 2.0
            tvec[2] = (a_tvec.flatten()[2] + m_tvec.flatten()[2]) / 2.0

        elif berm_position_one_obj_points.size != 0:
            a_ret, a_rvec, a_tvec = cv2.solvePnP(berm_position_one_obj_points, berm_position_one_all_corners, self.K, None)
            tvec.flatten()[0] = a_tvec.flatten()[0]
            tvec.flatten()[1] = a_tvec.flatten()[1]
            tvec.flatten()[2] = a_tvec.flatten()[2]

        elif berm_position_three_obj_points.size != 0:
            m_ret, m_rvec, m_tvec = cv2.solvePnP(berm_position_three_obj_points, berm_position_three_all_corners, self.K, None)
            tvec.flatten()[0] = m_tvec.flatten()[0]
            tvec.flatten()[1] = m_tvec.flatten()[1]
            tvec.flatten()[2] = m_tvec.flatten()[2]

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
