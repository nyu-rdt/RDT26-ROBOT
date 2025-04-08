import cv2
import apriltag
import numpy as np
from argparse import ArgumentParser

def main():
    parser = ArgumentParser(description='Test AprilTag Python bindings')
    # can specify if you want to use camera number or video file
    parser.add_argument('device_or_movie', metavar='INPUT', nargs='?', default=0,
                        help='Movie to load or integer ID of camera device')
    apriltag.add_arguments(parser)
    options = parser.parse_args()

    # define known tag sizes (in meters) based on ID
    # NEED ACTUAL TAG SIZES AND IDS
    tag_sizes = {
        1: 0.2,  # Tag ID 1 has a size of 0.2 meters
        2: 0.15,  # Tag ID 2 has a size of 0.15 meters
        3: 0.25   # Tag ID 3 has a size of 0.25 meters 
    }

    # initialize the camera
    cap = cv2.VideoCapture(int(options.device_or_movie)) if isinstance(options.device_or_movie, int) else cv2.VideoCapture(options.device_or_movie)
    if not cap.isOpened():
        print("Error: Could not open video.")
        return

    # creates a window to show the video 
    window = 'Camera'
    cv2.namedWindow(window)

    # initialize AprilTag detector with fine-tuned parameters
    detector_options = apriltag.DetectorOptions(
        quad_decimate=2.0,   # lower for better accuracy
        quad_sigma=0.0,      # applies guassian blur. adjust for noise reduction
        refine_edges=True    # enable edge refinement
    )

    detector = apriltag.Detector(detector_options)

    # UPDATE WITH REAL CALIBRATION DATA
    # calibration_data = cv2.calibrateCamera()
    fx, fy = 800, 800  # replace with camera's focal lengths
    cx, cy = 320, 240  # replace with camera's principal point
    camera_matrix = np.array([[fx, 0, cx],
                              [0, fy, cy],
                              [0, 0, 1]], dtype=float)

    dist_coeffs = np.zeros((4, 1))  # use actual distortion coefficients

    while True:
        # this takes a frame from the camera 
        success, frame = cap.read()
        if not success:
            break

        # converts the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        #finds any apriltags in the picture and returns them
        detections, dimg = detector.detect(gray, return_image=True)

        # how many tags were found 
        num_detections = len(detections)
        print(f'Detected {num_detections} tags.\n')

        for i, detection in enumerate(detections):
            tag_id = detection.tag_id  # get the tag ID number
            print(f'Detection {i + 1} of {num_detections}, ID: {tag_id}')

            print(f'Detection {i + 1} of {num_detections}:')
            print(detection.tostring(indent=2))

            #check if tag ID exists 
            if tag_id in tag_sizes:
                tag_size = tag_sizes[tag_id]
                print(f'Using tag size: {tag_size} meters')
            else:
                print(f'Unknown tag ID {tag_id}. Skipping.')
                continue

            # 3D coordinates of the tag corners in the tag's local coordinate frame
            object_points = np.array([
                [-tag_size / 2, -tag_size / 2, 0],
                [tag_size / 2, -tag_size / 2, 0],
                [tag_size / 2, tag_size / 2, 0],
                [-tag_size / 2, tag_size / 2, 0]
            ], dtype=float)

            # get corners of the tag
            image_points = detection.corners

            # solves for the position and orientation of the tag using the 3D points and the 2D image  points 
            # returns rvec, tvec 
            success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)

            if success:
                # calculates the straight lin edistance to the tag 
                distance = np.linalg.norm(tvec)
                print(f'Distance to tag: {distance:.2f} meters')

        cv2.imshow(window, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()

