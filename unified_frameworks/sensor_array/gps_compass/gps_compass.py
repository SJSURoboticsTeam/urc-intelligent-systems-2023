from typing import Tuple
import sys
root = __file__[: __file__.index("/unified_frameworks")]
sys.path.append(root + "/unified_frameworks")
from sensor_array.gps_compass.fake_gps_compass import FakeGPSCompass
from sensor_array.gps_compass.wireless_gps_compass import WirelessGPSCompass

try:
    _gps = WirelessGPSCompass()
    print("using wireless")
except Exception as e:
    print("exception e", e)
    _gps = FakeGPSCompass()
    print("using fake")


def geographic_coordinates_to_relative_coordinates(
    lat: float, long: float
) -> Tuple[float, float]:
    while _gps.get_cur_gps() is None:
        pass
    return _gps.geographic_coordinates_to_relative_coordinates(lat, long)


def disconnect():
    if isinstance(_gps, WirelessGPSCompass):
        _gps.disconnect()
