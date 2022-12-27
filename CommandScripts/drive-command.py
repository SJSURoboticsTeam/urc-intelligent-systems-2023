import sys, os
import requests
import serial.tools.list_ports as port_list
sys.path.insert(0, os.path.abspath(".."))
from modules.Serial import SerialSystem
import json
import time

port = "/dev/ttyUSB0"
get_initial_commands_url = "http://192.168.50.243:5000/drive"

web_response = requests.get(get_initial_commands_url)
print("Getting data from: " + web_response.text)

try:
    serial = SerialSystem(port, 38400)
    print("Using port: " + port)
except:
    ports = list(port_list.comports())
    print('====> Designated Port not found. Using Port:', ports[0].device)
    port = ports[0].device
    serial = SerialSystem(port, 38400)

homing_end = "Starting control loop..."
while True:
    response = serial.read_serial()
    if homing_end in response:
        while True:
            web_response = requests.get(get_initial_commands_url)
            response += serial.read_serial()
            if response != "No data received":
                serial.write_serial(web_response.text)
            else:
                continue
            
