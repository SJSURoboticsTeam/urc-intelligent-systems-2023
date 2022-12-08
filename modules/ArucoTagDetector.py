import cv2
import numpy as np

class ArucoTagDetector():
    def __init__(self):
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()

    def detect(self, image : np.ndarray):
        """
        Detects Aruco tags in the image and returns the corners and ID for each tag.

        Corners is an np array of shape (n, 4, 2) where n is the number of tags detected.
        Each tag has the corners top-left, top-right, bottom-right, bottom-left in that order.
        These coordinates are pixel level coordinates (e.g. 0 <= x <= width) and are integers

        Ids is an np array of shape (n, ) where n is the number of tags detected.

        If no tags are detected, then n will just be 0 (so the shape of corners will be (0, 4, 2) and the shape of ids will be (0,))
        """

        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, self.aruco_dict, parameters=self.parameters)

        if len(corners) > 0:
            # corners is a tuple of np arrays with shape [1, 4, 2], so just concat them to an array with shape [n, 4, 2]
            corners = np.concatenate(corners, axis=0)
            # convert the corners to integers to make them easier to work with
            corners = corners.astype(np.int32)

            # ids is a 2d array for some reason, so flatten it
            ids = ids.flatten()

            return corners, ids
        else:
            return np.zeros((0, 4, 2), dtype=np.int32), np.zeros((0,), dtype=np.int32)