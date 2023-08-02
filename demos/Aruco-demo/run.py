import sys
import cv2
sys.path.append( '/Users/mymac/Developer/Robotics/intelligent_systems/urc-intelligent_systems-2022')
from modules.ArucoTagDetector import ArucoTagDetector


if __name__ == '__main__':
    # Create a new ArucoTagDetector
    arucoTagDetector = ArucoTagDetector()

    # NOTE: change this value to a different number to use a different camera
    camera_id = 0

    camera = cv2.VideoCapture(camera_id)

    # check if the camera is open
    if not camera.isOpened():
        raise ValueError(f'Could not open camera {camera}')

    # loop until the user presses q
    while True:
        # read the image from the camera
        ret, image = camera.read()
        # detect the tags
        corners, ids = arucoTagDetector.detect(image)

        # loop through the tags and draw them using opencv
        for markerCorner, id in zip(corners, ids):
            # polyLine takes an iterable of points, and markerCorner is a 2d array of top-left, top-right, bottom-right, bottom-left points
            cv2.polylines(image, [markerCorner], True, (0, 255, 0), 2)
            # draw the marker id
            cv2.putText(image, str(id), (markerCorner[0][0], markerCorner[0][1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # show the image
        cv2.imshow('aruco tag detector', image)

        # if the user presses q, then exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break




