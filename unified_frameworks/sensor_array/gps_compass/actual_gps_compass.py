from typing import Tuple
from gps_compass_class import GPSCompass
import sys

root = __file__[: __file__.index("/unified_frameworks")]
sys.path.append(root + "/modules")

import GPS
import LSCM303


class ActualGPSCompass(GPSCompass):
    def __init__(self) -> None:
        self.gps = GPS.gpsRead()
        self.cur_gps = None
        self.compass = LSCM303.Compass()

        # get initial position
        print("getting initial gps position")
        while self.cur_gps is None:
            self.cur_gps = self.gps.get_position()

    def get_cur_angle(self) -> float:
        return self.compass.get_heading()

    def get_cur_gps(self) -> Tuple[int, int]:
        temp = self.gps.get_position()
        if temp is not None:
            self.cur_gps = temp
        return self.cur_gps
