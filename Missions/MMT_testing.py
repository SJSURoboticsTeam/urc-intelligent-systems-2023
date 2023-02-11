import os, sys
sys.path.insert(0, os.path.abspath(".."))
import requests
import serial.tools.list_ports as port_list
from modules.Serial import SerialSystem
from CommandScripts.autonomy import Autonomy
import json

port = "dev/ttyACM0"
baudrate = 38400
max_speed = 20
max_angle = 12
server = 'http://192.168.1.133:5000'
GPS_list = []

try:
    serial = SerialSystem(port, baudrate)
    print("Using port: " + port)
except:
    ports = list(port_list.comports())
    print('====> Designated Port not found. Using Port:', ports[0].device)
    port = ports[0].device
    serial = SerialSystem(port, baudrate)


json_str_example = '{"1": [-121.881073,37.335186], "2": [-121.881054,37.335132]}'
get_GPS_map_url = f"{server}/gps_map"
GPS_map = requests.get(get_GPS_map_url)
# GPS_map = json.loads(GPS_map.text)
GPS_map = json.loads(json_str_example)

for i in GPS_map:
    GPS_list.append(GPS_map[i])
print(GPS_list)

# example_GPS_map = [[-121.881073,37.335186],[-121.881054,37.335132]]

# rover = Autonomy(serial, f"{server}/drive", max_speed, max_angle, GPS_list)
# rover.start_mission()
