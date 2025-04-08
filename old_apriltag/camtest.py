from __future__ import division
from __future__ import print_function

from argparse import ArgumentParser
import cv2
import apriltag
import numpy as np

def main():
    '''Main function.'''

    parser = ArgumentParser(description='Test AprilTag Python bindings')
    parser.add_argument('device_or_movie', metavar='INPUT', nargs='?', default=0,
                        help='Movie to load or integer ID of camera device')
    apriltag.add_arguments(parser)
    options = parser.parse_args()

    # Initialize the camera
    cap = cv2.VideoCapture(int(options.device_or_movie)) if isinstance(options.device_or_movie, int) else cv2.VideoCapture(options.device_or_movie)

    if not cap.isOpened():
        print("Error: Could not open video.")
        return

    window = 'Camera'
    cv2.namedWindow(window)

    # Initialize AprilTag detector
    detector = apriltag.Detector(options, searchpath=apriltag._get_demo_searchpath())

    # Define camera intrinsic parameters (example values, adjust based on your camera)
    fx, fy = 800, 800  # Focal lengths
    cx, cy = 320, 240  # Principal point (image center)
    camera_matrix = np.array([[fx, 0, cx],
                               [0, fy, cy],
                               [0, 0, 1]], dtype=float)

    # Assuming no lens distortion
    dist_coeffs = np.zeros((4, 1))  # Adjust if you have distortion coefficients

    while True:
        success, frame = cap.read()
        if not success:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        detections, dimg = detector.detect(gray, return_image=True)

        num_detections = len(detections)
        print(f'Detected {num_detections} tags.\n')

        for i, detection in enumerate(detections):
            print(f'Detection {i + 1} of {num_detections}:')
            print(detection.tostring(indent=2))

            # Define the 3D coordinates of the tag corners in the tag's local coordinate frame
            #tag_size = 0.200  # Size of the tag (in meters, for example)
            tag_size = 0.2
            object_points = np.array([
                [-tag_size / 2, -tag_size / 2, 0],  # Bottom-left
                [tag_size / 2, -tag_size / 2, 0],   # Bottom-right
                [tag_size / 2, tag_size / 2, 0],    # Top-right
                [-tag_size / 2, tag_size / 2, 0]     # Top-left
            ], dtype=float)

            # Get the image points (corners of the detected tag)
            image_points = detection.corners

            # Solve for the pose
            success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)

            if success:
                # Calculate distance from the camera to the tag
                distance = np.linalg.norm(tvec)  # Euclidean distance
                print(f'Distance to tag: {distance:.2f} meters')

                # Optionally: draw the tag's axis on the frame
                axis = np.float32([[tag_size / 2, tag_size / 2, 0], 
                                    [tag_size / 2, tag_size / 2, -0.1], 
                                    [-tag_size / 2, -tag_size / 2, -0.1],
                                    [-tag_size / 2, -tag_size / 2, 0]]).reshape(-1, 3)
                imgpts, _ = cv2.projectPoints(axis, rvec, tvec, camera_matrix, dist_coeffs)
                frame = cv2.drawContours(frame, [np.int32(imgpts)], -1, (0, 255, 0), -3)

        overlay = frame // 2 + dimg[:, :, None] // 2
        cv2.imshow(window, overlay)
        if cv2.waitKey(1) == 27:  # Exit on 'ESC'
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()