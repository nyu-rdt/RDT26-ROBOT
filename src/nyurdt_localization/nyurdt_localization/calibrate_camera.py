"""
calibrate_camera.py
Used for capturing images & calibrating a camera.
Use ENTER to take a photo and ESCAPE to stop taking photos.
Press any key to cycle through detections.
"""
import cv2
import numpy as np
from pprint import pprint
from typing import List, Union

ENTER_KEY_VALUE = 13
ESC_KEY_VALUE = 27

CHESSBOARD_INNER_COL_PTS = 9
CHESSBOARD_INNER_ROW_PTS = 6


def read_camera_feed(source: Union[int, str] = 0) -> List[np.ndarray]:
   """
   Read frames from the camera feed
   :param source: The source of the camera feed, can be int or string. Defaults to 0
   :return: A list of images that were taken by the user
   """
   # Initialize video capture
   cap = cv2.VideoCapture(source)
   if not cap.isOpened():
      return []
   
   cap.set(cv2.CAP_PROP_FRAME_WIDTH, 352)
   cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 288)

   print(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
   print(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

   # Begin reading from the camera until ESC key is hit
   frames = []
   user_input = ''
   while user_input != ESC_KEY_VALUE:
      # Attempt reading a frame from the camera
      ret, frame = cap.read()
      if not ret: continue

      # Show the frame
      cv2.imshow("Camera", frame)

      # Check if picture taken
      if user_input == ENTER_KEY_VALUE:
         frames.append(frame)

         # Show a flash
         flash = [[255, 255, 255] for _ in range(frame.shape[0] * frame.shape[1])]
         flash = np.asarray(flash).reshape((frame.shape[0], frame.shape[1], 3))
         flash = flash.astype(np.uint8)
         flash = cv2.addWeighted(frame, 1, flash, 0.5, 0)

         cv2.imshow("Camera", flash)
         cv2.waitKey(100)

      # Update current input
      user_input = cv2.waitKey(1) & 0xFF

   # Release & return
   cap.release()
   cv2.destroyWindow("Camera")
   return frames


def calibrate(frames: List[np.ndarray]) -> None:
   """
   Calibrate given from images from a camera feed.
   :param frames: A list of images, all taken with the same camera
   :return: None
   """
   # Setup criteria & variables for calibration
   criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
   objp = np.zeros((CHESSBOARD_INNER_ROW_PTS*CHESSBOARD_INNER_COL_PTS,3), np.float32)
   objp[:,:2] = np.mgrid[0:CHESSBOARD_INNER_ROW_PTS,0:CHESSBOARD_INNER_COL_PTS].T.reshape(-1,2)

   objpoints = [] # 3d point in real world space
   imgpoints = [] # 2d points in image plane

   # Iterate over all frames
   for frame in frames:
      # Attempt to detect chessboard corners
      gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
      ret, corners = cv2.findChessboardCorners(gray, (CHESSBOARD_INNER_ROW_PTS, CHESSBOARD_INNER_COL_PTS), None)
      if ret:
         # Add detected points
         objpoints.append(objp)
         corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
         imgpoints.append(corners2)

         # Draw and display the corners
         cv2.drawChessboardCorners(frame, (CHESSBOARD_INNER_ROW_PTS, CHESSBOARD_INNER_COL_PTS), corners2, ret)
      cv2.imshow("Captured", frame)
      cv2.waitKey(0)

   # Check if no chessboard was detected in any image
   if not objpoints:
      print("\n-- No chessboard detected. Aborting calibration. --")
      return

   # Calibrate the camera
   ret, mtx, dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

   # # Print output if calibration successful
   if ret:
      print("DISTORTION COEFFICIENTS:")
      print(dist.tolist())

      print()
      print("=================================")
      print()

      print("CALIBRATION MATRIX")
      pprint(mtx.tolist())

   # print("\n=== Calibration Results ===")
   # print("RMS Reprojection Error:", ret)

   # print("\nDISTORTION COEFFICIENTS:")
   # print(dist.tolist())

   # print("\nCALIBRATION MATRIX:")
   # pprint(mtx.tolist())

   # print("\nfx =", mtx[0,0])
   # print("fy =", mtx[1,1])
   # print("cx =", mtx[0,2])
   # print("cy =", mtx[1,2])


def main(args=None):
   frames = read_camera_feed(0)
   calibrate(frames)


if __name__ == "__main__":
   main()