import sys
import serial.tools.list_ports as port_list
sys.path.append( '../../modules/Serial' )
from Serial import SerialSystem

port = "/dev/ttyUSB1"


try:
    serial = SerialSystem(port, 38400)
    print("Using port: " + port)
except:
    ports = list(port_list.comports())
    print('====> Designated Port not found. Using Port:', ports[0].device)
    port = ports[0].device
    serial = SerialSystem(port, 38400)

serial.read_serial()
message = input("Write something to the serial to read\n")
serial.write_serial(message)
while True:
    serial.read_serial()