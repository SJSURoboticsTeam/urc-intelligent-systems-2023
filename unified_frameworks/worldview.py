"""
This code is responsible for reading in all sensory inputs 
and unifying them into a single worldview for everything else
to access
"""

import time
from threading import Thread
from math import pi
from unified_utils import polar_sum, _Abstract_Service
import unified_frameworks.sensor_array.lidar.lidar
import bridge.client_side
import importlib
from sensor_array.gps_compass.bridge_gps import BridgeGPS
from typing import Tuple
from bridge import client_side

importlib.reload(bridge.client_side)
importlib.reload(unified_frameworks.sensor_array.lidar.lidar)
lidar = unified_frameworks.sensor_array.lidar.lidar


config = {
    "update_frequency": 20,  # Hz
    "service_event_verbose": True,
    "rover_body": [(-pi / 2, 1), (pi, 0.5), (0, 0.5)],
    "verbose": True,
}


class Worldview(_Abstract_Service):
    def __init__(self) -> None:
        """Set up all the sensors to feed their data into this worldview."""
        super().__init__()
        client_side.service.start_service()
        time.sleep(3)
        if config["verbose"]:
            print("Initializing Worldview")
        self._lidar = lidar.Lidar()
        self._gps = BridgeGPS()

    def start_service(self):
        """Start sensors and start unifying data from all sensors"""
        if config["verbose"]:
            print("Starting worldview service")
        self._lidar.start_service()

    def stop_service(self):
        """Stop unifying data and stop all sensors"""
        if config["verbose"]:
            print("Stopping worldview service")
        self._lidar.stop_service()
        self._gps.disconnect()
        client_side.service.stop_service()

    def get_obstacles(self):
        """Get a list of obstacles where each obstacle is a point cloud of the
        obstacle

        """
        return self._lidar.get_obstacles()

    def get_points(self):
        """Get a point cloud of the environment. The point cloud is a list of
        ``(radians, meters)``

        """
        return sum(self._lidar.get_point_clouds(), [])

    def get_rover_body(self):
        """Get the coordinates of the rovers body"""
        return config["rover_body"]

    def geographic_coordinates_to_relative_coordinates(
        self, target_latitude: float, target_longitude: float
    ) -> Tuple[float, float]:
        return self._gps.geographic_coordinates_to_relative_coordinates(
            target_latitude, target_longitude
        )


if __name__ == "__main__":
    w = Worldview()
    w.start_service()
    time.sleep(10)
    w.stop_service()
