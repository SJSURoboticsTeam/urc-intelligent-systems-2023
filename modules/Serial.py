import serial
import time
import json

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
        self.ser.write(message.encode())

    def read_serial(self):
         # wait for the Arduino to finish receiving the data
        while self.ser.out_waiting:
            pass
        # read the response from the Arduino, line by line
        response = ""
        while True:
            line = self.ser.readline().decode(errors='ignore')
            if "{" in line:
                response += line
            if "}" in line:
                response += line
                break
        return response

    def read_write_serial(self, message):
         # wait for the Arduino to finish receiving the data
        while self.ser.out_waiting:
            pass
        # read the response from the Arduino, line by line
        response = ""
        while True:
            line = self.ser.readline().decode(errors='ignore')
            if "{" in line:
                response += line
            if "}" in line:
                response += line
                break
        self.ser.write(message)
            

    def close_serial(self):
        self.ser.close()

import requests
port = "/dev/ttyUSB0"
get_initial_commands_url = "http://192.168.1.133:5000/drive"
import serial.tools.list_ports as port_list
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
while True:
    # commands = [0,0,0,'D',50,0]
    # json_command = {"HB":commands[0],"IO":commands[1],"WO":commands[2],"DM":f"{commands[3]}","CMD":[commands[4],commands[5]]}
    # json_data = json.dumps(json_command)
    # final_json = f"{json_data}\n"
    # final_json = final_json.replace(" ", "")
    # print(final_json)
    web_response = requests.get(get_initial_commands_url)
    # serial.read_write_serial(json_data.encode())
    print(serial.read_serial())
    # time.sleep(0.45)
    serial.write_serial(web_response.text)
