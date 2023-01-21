import cv2
import numpy as np
from collections import deque
from collections.abc import Iterable
from typing import Union
import math


# calibration = np.load('../calibration.npz')
# mtx = calibration['mtx']
# dist = calibration['dist']
# MARKER_SIZE_CM = 15
#
# def distance_to_tags(tag_corners):
#     # get the translation vectors from the camera to each tag and return it as a list
#     rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(tag_corners, MARKER_SIZE_CM, mtx, dist)
#     return tvecs


def translate_lat_lon(lat, lon, tvec, heading):
    # Convert the heading to radians and get the x and y components of the distance
    heading = np.deg2rad(heading)

    # combine the compass heading with the tvec to get the distance in x and y
    x = tvec[0] * np.cos(heading) - tvec[1] * np.sin(heading)
    y = tvec[0] * np.sin(heading) + tvec[1] * np.cos(heading)

    R_M = 6371000
    delta_lat = np.tan(y / R_M) * 180 / np.pi

    # get the radius of the earth at the camera's current latitude
    radius = R_M * np.cos(np.deg2rad(lat))
    delta_lon = np.tan(x / radius) * 180 / np.pi

    return lat + delta_lat, lon + delta_lon

class ArucoTagDetector():
    def __init__(self):
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()

    def detect(self, image : np.ndarray, tag_ids : Union[int, Iterable] = None):
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
            # ids is a 2d array for some reason, so flatten it
            ids = ids.flatten()

            if tag_ids is not None and not isinstance(tag_ids, Iterable):
                tag_ids = [tag_ids]

            # filter out any corners that don't match the tag_id
            if tag_ids is not None:
                corners = (corners[i] for i in range(len(ids)) if ids[i] in tag_ids)
                # also filter the ids
                ids = np.array(ids[i] for i in range(len(ids)) if ids[i] in tag_ids)

            # corners is a tuple of np arrays (with the shape [1, 4, 2] for some reason), so concat them to an array with shape [n, 4, 2]
            corners = np.concatenate(corners, axis=0).astype(np.int32)

            return corners, ids
        else:
            return np.zeros((0, 4, 2)), np.zeros((0,))

class ArucoTagAutonomy():

    def __init__(self, cap, num_frames_to_search):
        self.cap = cap

        self.detector = ArucoTagDetector()

        self.target_tag = 1

        self.num_frames = num_frames_to_search

    def search_for_tags(self):
        frames = []

        for i in range(self.num_frames):
            _, frame = self.cap.read()
            frames.append(frame)

        for frame in reversed(frames): # look at the most recent frame first
            tag_ids = [4, 5] if self.target_tag == 4 else self.target_tag # posts 4 and 5 are the gate, so we want to look for both of them

            corners, ids = self.detector.detect(frame, tag_ids=tag_ids)
            if len(corners) > 0:
                return corners, ids

        return [], [] # if we don't find the tags we're looking for in any of the frames, return two empty lists so we can check len(corners) to determine if the tags were found



if __name__ == '__main__':
    # test the accuracy of translate_lat_lon function at lattitude 37.33 and longitude -121.88 for various tvecs and headings
    def get_distance(lon1, lat1, lon2, lat2):

        R_KM = 6373.0
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)

        dis_lon = lon2 - lon1
        dis_lat = lat2 - lat1

        form1 = math.sin(dis_lat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dis_lon / 2) ** 2
        form2 = 2 * math.atan2(math.sqrt(form1), math.sqrt(1 - form1))

        distanceKM = R_KM * form2

        return distanceKM * 1000

    lat = 37.33
    lon = -121.88

    tvecs = np.array([[3, 4],
                      [6, 8],
                      [1, 2],
                      [.5, .5]])

    headings = np.array([0, 90, 180, 270])
    errors = []
    for heading in headings:
        for tvec in tvecs:
            pred_lat, pred_lon = translate_lat_lon(lat, lon, tvec, heading)
            pred_distance = get_distance(lon, lat, pred_lon, pred_lat)
            true_distance = np.linalg.norm(tvec)

            print(f'Heading: {heading}, tvec: {tvec}, pred_distance: {pred_distance}, true_distance: {true_distance}')
            print(f'Error: {pred_distance - true_distance}, Percent Error: {(pred_distance - true_distance) / true_distance * 100}%')
            errors.append((pred_distance - true_distance) / true_distance * 100)

    print(f'Average Percent Error: {np.mean(errors)}%')





