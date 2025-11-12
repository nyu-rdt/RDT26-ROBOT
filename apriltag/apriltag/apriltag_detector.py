#thanks Celestino

import cv2
import numpy as np
from pupil_apriltags import Detector

# === CAMERA INTRINSICS ===
#Logitech c615 bad wire camera
fx, fy = 473.8791885468119744, 473.6890399956254214
cx, cy = 507.3397801369775948, 344.0735124444565827
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0,  0,  1]])
dist_coeffs = np.array([-6.040749085709139210e-03 ,-7.749846321697238414e-04, -4.293201829939494369e-04, 7.274042688794011939e-03, -1.841876380914878085e-03]) 

# TAG SIZING
tag_sizes = { #meters
    18 : .069,
    19 : .245,
    20 : .196, 
    21 : .069
}

def world_tag_corners(tag_size):
    return np.array([[-tag_size/2, tag_size/2, 0], #top left 
                     [tag_size/2, tag_size/2, 0], #top right
                     [tag_size/2, -tag_size/2, 0], #bottom right
                     [-tag_size/2, -tag_size/2, 0]]) #bottom left

# === AXIS FOR VISUALIZATION ===
def axis(tag_size):
    return np.array([[0, 0 , 0],    #origin
                    [tag_size/2, 0, 0],
                    [0, tag_size/2, 0],
                    [0, 0, -tag_size/2],
                    ])


# === DETECTOR INITIALIZATION ===
detector = Detector(families='tag36h11', 
                    nthreads=2, 
                    quad_decimate=1.5, #higher -> faster, lower -> more accurate
                    quad_sigma= 0.0, #edge detection blur, unnecessary for good quality webcam images
                    refine_edges=True, 
                    decode_sharpening=0.25) #contrast

# === AXIS FOR VISUALIZATION ===

# IMAGE CAPTURE
# MAX_HEIGHT = 1000
# MAX_WIDTH = 2000

# print("loading image...")
# image = cv2.imread("apriltag_images/apriltag.jpeg")

# height, width = image.shape[:2]
# scale = min(MAX_HEIGHT / height, MAX_WIDTH / width, 1) 
# image = cv2.resize(image, (0,0), fx=scale,fy=scale) 
# gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# detections = detector.detect(gray)

# for det in detections:
#     tag_id = det.tag_id

#     corners = np.array(det.corners, np.int32)
#     cv2.polylines(image, [corners], True, (0, 255, 0), 2)
#     cv2.putText(image, f"ID {tag_id}", tuple(corners[0]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
    
#     tag_size = tag_sizes[tag_id]
#     object_points = np.float32(world_tag_corners(tag_size))
#     image_points = np.float32(corners)
#     retval, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)

#     points_2d, _ = cv2.projectPoints(axis(tag_size), rvec, tvec, camera_matrix, dist_coeffs)

#     #cv2.drawFrameAxes(axis(tag_size), camera_matrix, dist_coeffs, rvec, tvec, .5)

#     cv2.line(image, tuple(np.int32(points_2d[0].ravel())), tuple(np.int32(points_2d[1].ravel())), (0, 0 , 255), 3) #X -red
#     cv2.line(image, tuple(np.int32(points_2d[0].ravel())), tuple(np.int32(points_2d[2].ravel())), (0, 255 , 0), 3) #Y -green
#     cv2.line(image, tuple(np.int32(points_2d[0].ravel())), tuple(np.int32(points_2d[3].ravel())), (255, 0 , 0), 3) #X -blue


# cv2.imshow("Displayed Apriltags", image)
# cv2.waitKey(0)

# VIDEO FEED
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("can't open video")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("issue with frame!")
        break
    else:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detections = detector.detect(gray)

        for det in detections:
            print(tag_id)
            tag_id = det.tag_id

            corners = np.array(det.corners, np.int32)
            cv2.polylines(frame, [corners], True, (0, 255, 0), 2)
            cv2.putText(frame, f"ID {tag_id}", tuple(corners[0]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            
            tag_size = tag_sizes[tag_id]
            object_points = np.float32(world_tag_corners(tag_size))
            image_points = np.float32(corners)
            retval, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)

            points_2d, _ = cv2.projectPoints(axis(tag_size), rvec, tvec, camera_matrix, dist_coeffs)

            #cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, .5) 

            #DRAW AXIS
            cv2.line(frame, tuple(np.int32(points_2d[0].ravel())), tuple(np.int32(points_2d[1].ravel())), (0, 0 , 255), 3) #X -red
            cv2.line(frame, tuple(np.int32(points_2d[0].ravel())), tuple(np.int32(points_2d[2].ravel())), (0, 255 , 0), 3) #Y -green
            cv2.line(frame, tuple(np.int32(points_2d[0].ravel())), tuple(np.int32(points_2d[3].ravel())), (255, 0 , 0), 3) #X -blue

        cv2.imshow("Displayed frame", frame)
        cv2.waitKey(1)

 
cv2.destroyAllWindows()