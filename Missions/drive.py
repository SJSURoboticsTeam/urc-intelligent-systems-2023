import requests
from CommandScripts.autonomy import Autonomy


get_GPS_map_url = "http://192.168.50.243:5000/gps_map"
GPS_map = requests.get(get_GPS_map_url)
#example_GPS_map = [[-121.881073,37.335186],[-121.881054,37.335132]]

rover = Autonomy("/dev/ttyUSB0", 38400, "http://192.168.50.243:5000/drive", 20, 12, GPS_map.text)
rover.start_mission()

