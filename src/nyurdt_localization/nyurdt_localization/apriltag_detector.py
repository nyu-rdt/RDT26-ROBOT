'''
Detects AprilTag boards created for the 2023-2024 season
Calculates the linear distances (x,y,z) & rotations (R, P, Y) between the tags and the cameras
'''


"""
Is this intended for use? What's the point in this if the apriltag_ros nodes exist? (and vice-versa?)
Also just seems like legacy code (2023-2024 season above ^), maybe it doesn't fit your needs anymore?
"""

from nyurdt_localization.tag_information import TAG_OBJECT_POINTS
import time

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import apriltag

from typing import List, Tuple

class AprilTagDetector:

    def __init__(self):

        # Video capture
        self.cap = cv2.VideoCapture("/dev/video0")

        # Calibration matrix
        self.K = np.array([
            [643.5364971797594, 0.0, 330.67767410964905],
            [0.0, 644.1072230798435, 265.5947092030194],
            [0.0, 0.0, 1.0]])

        self.options = apriltag.DetectorOptions(families='tag36h11')
        self.detector = apriltag.Detector(self.options)

        self.BERM_POSITION_ONE_IDS = set([7,8,9,10,11,12])
        self.BERM_POSITION_THREE_IDS = set([18,19,20,21])


    def get_pose(self, detections: List[apriltag.Detection]) -> Tuple[bool, np.ndarray, np.ndarray]:
        """
        Get the pose of detections relative to the camera

        :param detections: List of apriltag detections
        :return: Tuple, first item indicated success, second is translation & third is rotation
        """
        # Lists for detection positions
        berm_position_one_obj_points = []
        berm_position_one_all_corners = []

        berm_position_three_obj_points = []
        berm_position_three_all_corners = []
    
        # Filtering each detection into its appropriate list
        for tag in detections:
            # Get tag object & image points
            tag_points = TAG_OBJECT_POINTS[tag.tag_id]
            corners = np.array(tag.corners).astype(np.float32)

            # Check which board this detection belongs to
            if tag.tag_id in self.BERM_POSITION_ONE_IDS:
                berm_position_one_obj_points.extend(tag_points)
                berm_position_one_all_corners.extend(corners)
            elif tag.tag_id in self.BERM_POSITION_THREE_IDS:
                berm_position_three_obj_points.extend(tag_points)
                berm_position_three_all_corners.extend(corners)

        # Convert to numpy arrays
        berm_position_one_obj_points = np.asarray(berm_position_one_obj_points)
        berm_position_one_all_corners = np.asarray(berm_position_one_all_corners)

        berm_position_three_obj_points = np.asarray(berm_position_three_obj_points)
        berm_position_three_all_corners = np.asarray(berm_position_three_all_corners)

        # Process positions of each board
        pos_one_ret, pos_one_rvec, pos_one_tvec = self.process_points(
            berm_position_one_obj_points,
            berm_position_one_all_corners
        )

        pos_three_ret, pos_three_rvec, pos_three_tvec = self.process_points(
            berm_position_three_obj_points,
            berm_position_three_all_corners
        )

        # Get average pose between the two boards
        rvec = np.zeros((3,))
        tvec = np.zeros((3,))
        num_boards = 0

        if pos_one_ret:
            rvec = rvec + pos_one_rvec
            tvec = tvec + pos_one_tvec
            num_boards += 1
        if pos_three_ret:
            rvec = rvec + pos_three_rvec
            tvec = tvec + pos_three_tvec
            num_boards += 1

        # Terminate if no boards detected
        if num_boards == 0:
            return False, None, None

        rvec /= num_boards
        tvec /= num_boards

        # Convert to euler angles
        rmat, _ = cv2.Rodrigues(rvec)
        euler = R.from_matrix(rmat).as_euler('xyz', degrees=True)

        return True, tvec, euler
    

    def process_points(self, obj_points: np.ndarray, image_points: np.ndarray) -> Tuple[bool, np.ndarray, np.ndarray]:
        """
        Process two lists of corresponding points & get the translation & rotation

        :param obj_points: Real world coordinates
        :param image_points: Image points of apriltags in the image
        :return: Tuple, first item indicated success, second is translation & third is rotation
        """ 
        # Terminate if no points
        if obj_points.size <= 0:
            return False, None, None
        
        # Solve for rotation & translation
        ret, rvec, tvec = cv2.solvePnP(obj_points, image_points, self.K, None)
        
        if not ret:
            return False, None, None

        return True, rvec.flatten(), tvec.flatten()


    def draw_results(self, results: List[apriltag.Detection], image: np.ndarray) -> None:
        """
        Draw detections on an image
        
        :param results: List of apriltag detections
        :param image: Image to be displayed & drawn on
        :return: None
        """
        # For every detection...
        for r in results:
            # Get the corners in the image
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))

            # Draw a line between each corner
            cv2.line(image, ptA, ptB, (0, 255, 0), 2)
            cv2.line(image, ptB, ptC, (0, 255, 0), 2)
            cv2.line(image, ptC, ptD, (0, 255, 0), 2)
            cv2.line(image, ptD, ptA, (0, 255, 0), 2)

            # Draw a dot in the cnter
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)

        # Show the image
        cv2.imshow("Apriltag Detection", image)
        cv2.waitKey(1)
        time.sleep(0.1)


    def get_frame(self) -> Tuple[bool, np.ndarray]:
        """
        Capture a single image frame

        :return: Tuple, first item indicates success, second is the image frame
        """
        return self.cap.read()


    def get_detections(self, raw_image: np.ndarray) -> List[apriltag.Detection]:
        """
        Get apriltag detections from raw image

        :param raw_image: Image to be checked for detections
        :return: List of apriltag detections
        """
        # Get all results
        gray = cv2.cvtColor(raw_image, cv2.COLOR_BGR2GRAY)
        results_raw  = self.detector.detect(gray)

        # Filter out unused tags
        results = [tag for tag in results_raw if tag.tag_id in TAG_OBJECT_POINTS]
        return results


def main():

    aptag_node = AprilTagDetector()

    while True:
        ret, frame = aptag_node.get_frame()
        if not ret: continue

        detections = aptag_node.get_detections(frame)
        aptag_node.draw_results(detections, frame)

        if not detections: continue

        _, tvec, euler = aptag_node.get_pose(detections)

        print("Translation:", tvec)
        print("Rotation:", euler)


if __name__ == "__main__":
    main()