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
        self.ser.write(message.encode("utf-8"))

    def read_serial(self):
        c = self.ser.read()
        if len(c) == 0:
            print("No data received")
        else:
            try:
                c += self.ser.read(self.ser.inWaiting())
                print(c.decode("utf-8"))
            except:
                print("Error decoding data")
        time.sleep(0.4)
        return c.decode("utf-8")

    def close_serial(self):
        self.ser.close()