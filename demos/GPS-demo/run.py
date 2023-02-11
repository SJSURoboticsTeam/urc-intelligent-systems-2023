import sys
import time
import serial.tools.list_ports as port_list
sys.path.append( '../../')
from modules.GPS import gpsRead

if __name__ == '__main__':
    try:
        data = gpsRead("/dev/ttyACM1",9600)
        print("GPS Port found")
        url = "http://192.168.1.133:5000/gps"
    except:
            port_number = 0
            ports = list(port_list.comports())
            print('====> Designated Port not found. Using Port:', ports[port_number].device)
            port = ports[port_number].device
            data = gpsRead(port,9600)
            while data.get_position() == ['error', 'error'] or data.get_position() == ["None", "None"]:
                print("Port not found, going to next port...")
                port_number += 1
                port = ports[port_number].device
                try:
                    data = gpsRead(port,9600)
                    break
                except:
                    continue
    GPS_Coordinates = data.get_position(url)

    while True:
        print(GPS_Coordinates)
        time.sleep(1)