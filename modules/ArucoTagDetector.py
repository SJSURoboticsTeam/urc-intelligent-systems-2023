import cv2
import numpy as np
from collections.abc import Iterable
from typing import Union
import math

MARKER_SIZE_M = .15


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

            if tag_ids is not None and not isinstance(tag_ids, Iterable): # if tag_ids is an int, make it into a list so we can iterate over it
                tag_ids = [tag_ids]

            # filter out any corners that don't match the tag_id
            if tag_ids is not None:
                filtered_corners = []
                filtered_ids = []

                for i in range(len(ids)):
                    if ids[i] in tag_ids:
                        filtered_corners.append(corners[i])
                        filtered_ids.append(ids[i])

                corners = filtered_corners
                ids = np.array(filtered_ids)

            if len(corners) > 0: # need to check this again because we might have filtered out all the corners
                # corners is a tuple of np arrays (with the shape [1, 4, 2] for some reason), so concat them to an array with shape [n, 4, 2]
                corners = np.concatenate(corners, axis=0)

            return corners, ids
        else:
            return np.zeros((0, 4, 2)), np.zeros((0,))


class ArucoTagAutonomy():

    def __init__(self, cap, num_frames_to_search = 4):
        self.cap = cap

        self.detector = ArucoTagDetector()

        self.target_tags = 1

        self.num_frames = num_frames_to_search

    def search_for_tags(self):
        # the detector doesn't always find the tags, even when standing still, so search for the tag in a few frames first

        for _ in range(self.num_frames):
            _, frame = self.cap.read()  
            if not frame:
                print('No frame, moving to next one')
                continue

            corners, ids = self.detector.detect(frame, tag_ids=self.target_tags)
            if len(corners) > 0:
                return corners, ids

        return [], [] # if we don't find the tags we're looking for in any of the frames, return two empty lists so we can check len(corners) to determine if the tags were found
        #return False

    def search_for_post(self):
        """
        Searches for a post marked by Aruco tags matching the target tag and returns the translation vector (x, y, z) from the camera to the post (in meters).
        If the target tag is the gate, then the translation vector is the average of the two posts (assuming the tags of both gate posts are in sight, otherwise, it is just the translation vector to the post that is in sight).
        If no aruco tags are detected, returns None.
        """
        post_tvec = None

        corners, ids = self.search_for_tags()

        if len(corners) > 0:
            tvecs = self.distance_to_tags(corners)

            post_tvec = self.calculate_target_translation(tvecs, ids)

        return post_tvec

    def distance_to_tags(self, tag_corners):

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(tag_corners, MARKER_SIZE_M, self.cap.mtx, self.cap.dist)
        tvecs = tvecs[:, 0, :] # for some reason, cv2 returns [n, 1, 3] where n is the number of tags detected, so we need to remove the extra dimension
        return tvecs

    def update_target_tag(self):
        """
        After the rover has reached a post, the target aruco tag needs to be updated to the next post.
        If the rover has reached the last post (with the tag 3), then the target tag needs to be updated to the gate (tags 4 and 5).
        """
        if self.target_tags == 3:
            self.target_tags = [4, 5]
        else:
            self.target_tags += 1

    @staticmethod
    def translate_lat_lon(lat, lon, tvec, heading):
        # Convert the heading to radians and get the x and y components of the distance
        heading = np.deg2rad(heading)

        # combine the compass heading with the tvec to get the distance in x and y
        x = tvec[0] * np.cos(heading) - tvec[1] * np.sin(heading)
        y = tvec[0] * np.sin(heading) + tvec[1] * np.cos(heading)

        R_M = 6371000
        delta_lat = np.tan(y / R_M) * 180 / np.pi

        # get the radius of the earth at the camera's current latitude (needed to calculate the change in longitude)
        radius = R_M * np.cos(np.deg2rad(lat))
        delta_lon = np.tan(x / radius) * 180 / np.pi

        return lat + delta_lat, lon + delta_lon

    @staticmethod
    def calculate_target_translation(tvecs: np.ndarray, ids: np.ndarray) -> np.ndarray:
        """
        Calculates the distance (x, y, z) between the target aruco tag post and the current location of the rover.
        If we're dealing with the gate, then we just use the average of the two posts (assuming both posts are visible).

        :param tvecs: an array of translation with the shape (n, 3) between the rover and each individual tag.
        :param ids: the ID of each aruco tag with the shape (n, 1).
        :return: a numpy array of shape (1, 3) containing the (x, y, z) distance between the target aruco tag post and the current location of the rover.

        """

        unique_ids = np.unique(ids)
        post_locations = np.zeros((len(unique_ids), 3))

        for i, unique_id in enumerate(unique_ids):
            indices = np.nonzero(ids == unique_id)[0]
            if len(indices) > 1:
                # Calculate the average of the tvecs for the tags with the same id
                post_location = np.mean(tvecs[indices], axis=0)
            else:
                # Use the tvec for the single tag with this id as the post location
                post_location = tvecs[indices[0]]
            post_locations[i] = post_location

        # Calculate the target location as the average of all post locations
        target_location = np.mean(post_locations, axis=0)

        return target_location





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
            pred_lat, pred_lon = ArucoTagAutonomy.translate_lat_lon(lat, lon, tvec, heading)
            pred_distance = get_distance(lon, lat, pred_lon, pred_lat)
            true_distance = np.linalg.norm(tvec)

            print(f'Heading: {heading}, tvec: {tvec}, pred_distance: {pred_distance}, true_distance: {true_distance}')
            print(f'Error: {pred_distance - true_distance}, Percent Error: {(pred_distance - true_distance) / true_distance * 100}%')
            errors.append((pred_distance - true_distance) / true_distance * 100)

    print(f'Average Percent Error: {np.mean(errors)}%')





