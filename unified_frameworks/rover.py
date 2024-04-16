"""
This module is a script that starts all sensors on the
rover side. When executing the file, the process runs infinitely
and must be keyboard-interrupted in order to properly halt.
"""

import time
from bridge import rover_side
from sensor_array.lidar.bridge_lidar import BridgeLidar
from sensor_array.gps_compass.bridge_gps import BridgeGPS

if __name__ == "__main__":
    rover_side.service.start_service()
    time.sleep(1)
    lidar = BridgeLidar()
    gps = BridgeGPS()
    lidar.connect()
    gps.connect()
    time.sleep(3)  # give time for threads to start
    try:
        while 1:
            print(rover_side.data.keys())
            time.sleep(1)
    except KeyboardInterrupt:
        print("closing rover sensors")
        lidar.disconnect()
        gps.disconnect()
        rover_side.service.stop_service()
