import sys
import requests
import serial.tools.list_ports as port_list
sys.path.append( '../modules/Serial')
from Serial import SerialSystem
import json

port = "/dev/ttyUSB0"
get_initial_commands_url = "http://192.168.50.243:5000/drive"

def jsonParse(str):
    if(str.count("{") >= 1 and str.count("}") >= 1 ):
        if(str.rfind("{") < str.rfind("}")):
            return str[str.rfind("{"):str.rfind("}")+1]
        else:
            return str[str.rfind("{", 0, str.rfind("{")):str.rfind("}")+1]

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
        print("Parsing Data from Pi...")
        while True:
            serial.write_serial(web_response.text)
            response += serial.read_serial()
            json_format = jsonParse(response)

            if json_format != None:
                response = ''
                json_format = json.loads(json_format)
                web_response = requests.get(get_initial_commands_url, params=json_format)


