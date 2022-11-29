from math import floor
from rplidar import RPLidar

PORT_NAME = 'COM6'
lidar = RPLidar(PORT_NAME)

def process_data(data):
    for angle in range(360):
        distance = data[angle]
        print(distance)
    data = [x for x in data if x != 0]
    min_distance = min(data)
    max_distance = max(data)
    print("Minimum distance: ", min_distance)
    print("Maximum distance: ", max_distance)

scan_data = [0]*360

try:
    for scan in lidar.iter_scans(max_buf_meas=10000):
        for (_, angle, distance) in scan:
            scan_data[min([359, floor(angle)])] = distance
        process_data(scan_data)

except KeyboardInterrupt:
    print('Stoping.')
lidar.stop()
lidar.stop_motor()
lidar.disconnect()