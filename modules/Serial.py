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
        try:
            self.ser.write(message.encode('utf-8'))
            time.sleep(0.1)
            while self.ser.inWaiting()==0: pass
            if  self.ser.inWaiting()>0:
                response = self.ser.readline()
                response = response.decode("utf-8")
                print(response)
                self.ser.flushInput()
        except KeyboardInterrupt:
            print("KeyboardInterrupt has been caught.")
        return response
            

    def close_serial(self):
        self.ser.close()


