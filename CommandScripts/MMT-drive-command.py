import sys
import time
import requests
import json
import serial.tools.list_ports as port_list
import os, sys
sys.path.insert(0, os.path.abspath(".."))
from modules.Serial import SerialSystem

port = "/dev/ttyUSB0"
get_initial_commands_url = "http://192.168.1.133:5000/drive"

# web_response = requests.get(get_initial_commands_url)
# print("Getting data from: " + web_response.text)

try:
    serial = SerialSystem(port, 38400)
    print("Using port: " + port)
except:
    ports = list(port_list.comports())
    print('====> Designated Port not found. Using Port:', ports[0].device)
    port = ports[0].device
    serial = SerialSystem(port, 38400)

print("Starting Serial...")
time.sleep(3)
while True:
    commands = [0,0,0,'D',50,0]
    json_command = {"HB":commands[0],"IO":commands[1],"WO":commands[2],"DM":f"{commands[3]}","CMD":[commands[4],commands[5]]}
    json_data = json.dumps(json_command)
    # final_json = f"{json_data}\n"
    # web_response = requests.get(get_initial_commands_url)
    # serial.read_write_serial(json_data.encode())
    print(serial.read_serial())
    serial.write_serial("Hello")
    # time.sleep(0.45)
    # serial.write_serial(web_response.text)
    #serial.read_write_serial(web_response.text)
    # serial.write_serial(web_response.text)