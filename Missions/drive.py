import os, sys

sys.path.insert(0, os.path.abspath(".."))
import requests
import serial.tools.list_ports as port_list
from modules.WiFi import WiFi
from CommandScripts.autonomy import Autonomy
from modules.GPS import gpsRead
import json
import time

serial_port = "/dev/ttyACM2"
gps_port = "/dev/ttyACM0"
serial_baudrate = 38400
gps_baudrate = 57600
max_speed = 50
max_angle = 12
server = "http://192.168.0.211:5000"
GPS_list = []
rover_comms = WiFi(server)


try:
    GPS = gpsRead(gps_port, gps_baudrate)
    print("Using port: " + gps_port, "For GPS")
except:
    port_number = 0
    ports = list(port_list.comports())
    print(
        "====> Designated Port not found. Using Port:",
        ports[port_number].device,
        "For GPS Connection",
    )
    port = ports[port_number].device
    GPS = gpsRead(port, gps_baudrate)
    while GPS.get_position() == ["error", "error"] or GPS.get_position() == [
        "None",
        "None",
    ]:
        print("Port not found, going to next port...")
        port_number += 1
        gps_port = ports[port_number].device
        try:
            GPS = gpsRead(port, gps_baudrate)
        except:
            continue
        break


# GPS_map_url = f"{server}/gps_map"
# try:
#     GPS_map = requests.get(GPS_map_url)
# except:
#     print("Could not get GPS map from mission control")
#     exit(1)

# GPS_map = json.loads(GPS_map.text)

# for i in GPS_map:
#     GPS_list.append(GPS_map[i])

GPS_list = [
    [-121.8818545, 37.3370768],
    [-121.8818535, 37.3370083],
    [-121.8817744, 37.3369864],
]
print("GPS List:", GPS_list)

try:
    rover = Autonomy(rover_comms, server, max_speed, max_angle, GPS, GPS_list)

    # start mission - you need to control c your program
    rover.start_mission()
except KeyboardInterrupt:
    print("stopping...")
except Exception as e:
    raise
finally:
    rover.rover_comms.write_data(
        {"HB": 0, "IO": 1, "WO": 0, "DM": "D", "CMD": [0, 0], "LS": 3}
    )
    print("finished stopping")
    quit()
