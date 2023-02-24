import sys
import serial.tools.list_ports as port_list
sys.path.append( '../../')
from modules.Serial import SerialSystem
import json

port = "/dev/ttyUSB0"

try:
    serial = SerialSystem(port, 38400)
    print("Using port: " + port)
except:
    ports = list(port_list.comports())
    print('====> Designated Port not found. Using Port:', ports[0].device)
    port = ports[0].device
    serial = SerialSystem(port, 38400)

speeds = [50, 45, 20, 11, 77, 32, 88]
while True:
    for i in speeds:
        data = {"HB":0,"IO":1,"WO":0,"DM":"D","CMD":[0,i]}
        data = json.dumps(data)
        data = data.replace(" ", "")
        print("Here")
        answer = serial.read_serial()
        serial.write_serial(data)

        

    