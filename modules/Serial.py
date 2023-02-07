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

    def read_write_serial(self, message):
        line = self.ser.read(self.ser.inWaiting())
        print(line.decode("utf-8"))
        time.sleep(0.10)
        if (len(line) != 0):
            self.ser.write(message.encode('utf-8'))
            

    def close_serial(self):
        self.ser.close()