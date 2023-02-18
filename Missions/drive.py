import os, sys
sys.path.insert(0, os.path.abspath(".."))
import requests
import serial.tools.list_ports as port_list
from modules.Serial import SerialSystem
from CommandScripts.autonomy import Autonomy

port = "dev/ttyACM0"
baudrate = 38400
max_speed = 20
max_angle = 12

try:
    serial = SerialSystem(port, baudrate)
    print("Using port: " + port)
except:
    ports = list(port_list.comports())
    print('====> Designated Port not found. Using Port:', ports[0].device)
    port = ports[0].device
    serial = SerialSystem(port, baudrate)

get_GPS_map_url = "http://192.168.50.243:5000/gps_map"
GPS_map = requests.get(get_GPS_map_url)
# example_GPS_map = [[-121.881073,37.335186],[-121.881054,37.335132]]

rover = Autonomy(serial, "http://192.168.50.243:5000/drive", 20, 12, GPS_map.text)
rover.start_mission()
