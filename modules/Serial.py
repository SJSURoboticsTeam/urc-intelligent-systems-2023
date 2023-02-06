import serial
import time

class SerialSystem:
    def __init__(self, port, baudrate):
        
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(
                            port = port,
                            baudrate = baudrate,
                            parity = serial.PARITY_NONE,
                            stopbits = serial.STOPBITS_ONE,
                            bytesize = serial.EIGHTBITS,
                            )
        self.ser.rts = False
        self.ser.dtr = False
        self.ser.timeout = 0

    def write_serial(self, message):
        self.ser.write(message.encode('utf-8'))

    def read_serial(self, message):
        line = self.ser.read(self.ser.inWaiting())
        print(line.decode("utf-8"))
        time.sleep(0.25)
        if (self.ser.inWaiting()):
            self.ser.write(message.encode('utf-8'))

    def read_write_serial(self, message):
        line = self.ser.read()
        if len(line) != 0:
            try:
                line += self.ser.read(self.ser.inWaiting())
                print(line.decode("utf-8"))
            except:
                print("Error decoding data")
            #time.sleep(0.5)
            self.ser.write(message.encode('utf-8'))
            

    def close_serial(self):
        self.ser.close()


import serial.tools.list_ports as port_list
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

while True:
    data = {"HB":0,"IO":0,"WO":0,"DM":"D","CMD":[10,0]}
    serial.read_serial(json.dumps(data))