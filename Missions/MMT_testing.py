import os, sys
sys.path.insert(0, os.path.abspath(".."))
import requests
import serial.tools.list_ports as port_list
from modules.Serial import SerialSystem
from CommandScripts.autonomy import Autonomy
from modules.GPS import gpsRead
import json

port = "/dev/ttyACM0"
baudrate = 38400
max_speed = 5
max_angle = 12
server = 'http://192.168.1.133:5000'
GPS_list = []

try:
    serial = SerialSystem(port, baudrate)
    print("Using port: " + port, "For Serial Comms")
except:
    ports = list(port_list.comports())
    print('====> Designated Port not found. Using Port:', ports[0].device, "For Serial Connection")
    port = ports[0].device
    serial = SerialSystem(port, baudrate)


try:
    GPS = gpsRead("/dev/ttyACM1",9600)
    print("GPS Port found")
except:
        port_number = 0
        ports = list(port_list.comports())
        print('====> Designated Port not found. Using Port:', ports[port_number].device, "For GPS Connection")
        port = ports[port_number].device
        # GPS = gpsRead(port,9600)
        # while GPS.get_position() == ['error', 'error'] or GPS.get_position() == ["None", "None"]:
        #     print("Port not found, going to next port...")
        #     port_number += 1
        #     port = ports[port_number].device
        #     try:
        #         GPS = gpsRead(port,9600)
        #     except:
        #         continue
        #     break


# json_str_example = '{"1": [-121.881073,37.335186], "2": [-121.881054,37.335132]}'
json_str_example = '{"1": [-121.881073,37.335186]}'

# get_GPS_map_url = f"{server}/gps_map"
# GPS_map = requests.get(get_GPS_map_url)
# GPS_map = json.loads(GPS_map.text)
GPS_map = json.loads(json_str_example)

for i in GPS_map:
    GPS_list.append(GPS_map[i])
# print("GPS List", GPS_list)

example_GPS_map = [[-121.881073,37.335186],[-121.881054,37.335132]]
# example_GPS_map = [[-121.881073,37.335186]]

rover = Autonomy(serial, f"{server}/drive", max_speed, max_angle, GPS, example_GPS_map)
rover.start_mission()
