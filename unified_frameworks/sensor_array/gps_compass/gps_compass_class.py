from abc import ABC, abstractmethod
from typing import Tuple, Union
import math


class Util:
    """
    Methods were taken from AutoHelp
    """

    def get_distance(
        current_GPS: Tuple[float, float], target_GPS: Tuple[float, float]
    ) -> Union[Tuple[float, float], None]:
        """
        Returns a tuple containing the distance between current_GPS and target_GPS
        The tuple is of the following format: (distanceInKM, distanceInMiles)

        """
        R_KM = 6373.0
        R_MI = 3958.8
        try:
            current_lat = math.radians(current_GPS[1])
            current_lon = math.radians(current_GPS[0])
            target_lat = math.radians(target_GPS[1])
            target_lon = math.radians(target_GPS[0])

            dis_lon = target_lon - current_lon
            dis_lat = target_lat - current_lat

            form1 = (
                math.sin(dis_lat / 2) ** 2
                + math.cos(current_lat)
                * math.cos(target_lat)
                * math.sin(dis_lon / 2) ** 2
            )
            form2 = 2 * math.atan2(math.sqrt(form1), math.sqrt(1 - form1))

            distanceKM = R_KM * form2
            distanceMi = R_MI * form2
            return distanceKM, distanceMi
        except:
            print("No GPS Data")

    def get_bearing(current_GPS, target_GPS):
        """Returns the angle between two GPS coordinates
        
        PARAMS:
            current_GPS (tuple): (latitude, longitude)
            target_GPS (tuple):  (latitude, longitude)
        RETURNS:
            float. angle between the two coordinates
            
        """
        try:
            current_latitude = math.radians(current_GPS[1])
            current_longitude = math.radians(current_GPS[0])
            target_latitude = math.radians(target_GPS[1])
            target_longitude = math.radians(target_GPS[0])

            deltalog = target_longitude - current_longitude

            x = math.cos(target_latitude) * math.sin(deltalog)
            y = (math.cos(current_latitude) * math.sin(target_latitude)) - (
                math.sin(current_latitude)
                * math.cos(target_latitude)
                * math.cos(deltalog)
            )

            bearing = (math.atan2(x, y)) * (180 / 3.14)
            return bearing
        except:
            print("No GPS Data")


class GPSCompass(ABC):
    @abstractmethod
    def get_cur_angle(self) -> int:
        """
        Returns the current angle from north, in degrees
        """
        pass

    @abstractmethod
    def get_cur_gps(self) -> Tuple[int, int]:
        """
        Returns the current GPS position in the form (latitude, longitude)
        """
        pass

    def geographic_coordinates_to_relative_coordinates(
        self, target_latitude: float, target_longitude: float
    ) -> Tuple[float, float]:
        """
        Convert the target latitude/longitude into polar form of (theta, r)
        where theta is in radians
        """
        cur_angle = self.get_cur_angle()
        cur_gps = self.get_cur_gps()

        distance = Util.get_distance(cur_gps, (target_latitude, target_longitude))
        distance = distance[0] * 1000  # convert from km to m
        target_angle = Util.get_bearing(cur_gps, (target_latitude, target_longitude))

        # this is how much we need to turn
        relative_angle = cur_angle - target_angle
        # add 90* because the front of the rover is at rahul's 90* mark
        final_angle = relative_angle + 90
        # make positive angle
        if final_angle < 0:
            final_angle += 360

        # convert to radians before returning
        return (final_angle * math.pi / 180, distance)
