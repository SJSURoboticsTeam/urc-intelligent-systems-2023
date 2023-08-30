import cv2
import numpy as np
from collections import deque
import os
import glob
import depthai
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../'))
sys.path.append(parent_dir)
from modules.OAKD import OakD

# load the 4x4_50 aruco tag dictionary
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()

cap = OakD(depthai.ColorCameraProperties.SensorResolution.THE_720_P)

mtx = cap.mtx
dist = cap.dist

frame_queue = deque(maxlen=1)

while True:
    _, frame = cap.read()

    frame_queue.append(frame)

    corners, ids = [], []

    for test_frame in frame_queue:
        # detect the aruco markers in the image
        possible_corners, possible_ids, rejectedImgPoints = cv2.aruco.detectMarkers(test_frame, arucoDict,parameters=parameters)
        if len(possible_corners) > 0:
            corners.extend(possible_corners)
            ids.extend(possible_ids.flatten())
            break

    for corner, tag_id in zip(corners, ids):
        # loop over the detected ArUCo corners
        #for (markerCorner, markerID) in zip(corners, ids):
        # extract the marker corners (which are always returned in
        # top-left, top-right, bottom-right, and bottom-left order)
        # corners = markerCorner.reshape((4, 2))

        (topLeft, topRight, bottomRight, bottomLeft) = corner[0]
        # convert each of the (x, y)-coordinate pairs to integers
        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))
        # draw the bounding box of the ArUCo detection
        cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
        cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
        cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
        cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
        # compute and draw the center (x, y)-coordinates of the ArUco
        # marker
        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
        # draw the ArUco marker ID on the frame
        cv2.putText(frame, str(tag_id),
            (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
            0.5, (0, 255, 0), 2)

        # estimate the distance of the marker from the camera
        markerSizeInM = .20

        rvect, tvect, _ = cv2.aruco.estimatePoseSingleMarkers(corner, markerSizeInM, mtx, dist)

        distance = np.linalg.norm(tvect[0][0])
        # show the distance in the frame (limit the distance to 2 decimal places))
        cv2.putText(frame, str(distance), (topLeft[0], topLeft[1] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    if frame is not None:

        #frame = cv2.resize(frame, None, fx=.25, fy=.25)
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF
        #if the `q` key was pressed, break from the loop
        if key == ord("q"):
           break
       
    # close all windows
    cv2.destroyAllWindows()