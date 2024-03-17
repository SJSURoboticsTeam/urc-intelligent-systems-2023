from typing import Tuple
import sys
root = __file__[: __file__.index("/unified_frameworks")]
sys.path.append(root + "/unified_frameworks")
from sensor_array.gps_compass.gps_compass_class import GPSCompass
import math


class FakeGPSCompass(GPSCompass):
    def __init__(self) -> None:
        super().__init__()

    def get_cur_angle(self) -> int:
        pass

    def get_cur_gps(self) -> Tuple[int, int]:
        pass

    def geographic_coordinates_to_relative_coordinates(
        self, target_latitude: float, target_longitude: float
    ):
        return (math.pi / 2, 4)  # arbitrary values
