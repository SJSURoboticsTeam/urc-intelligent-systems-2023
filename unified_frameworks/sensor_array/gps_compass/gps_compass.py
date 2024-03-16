from typing import Tuple
from fake_gps_compass import FakeGPSCompass
from wireless_gps_compass import WirelessGPSCompass

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
    return _gps.geographic_coordinates_to_relative_coordinates(lat, long)
