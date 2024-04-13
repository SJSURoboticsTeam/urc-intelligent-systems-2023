import time
from bridge import rover_side
from sensor_array.bridge_lidar import BridgeLidar
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
            print(rover_side.data)
            time.sleep(1)
    except KeyboardInterrupt:
        print("closing rover sensors")
