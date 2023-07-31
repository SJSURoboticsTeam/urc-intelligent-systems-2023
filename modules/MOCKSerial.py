import random
import time

class MockSerialSystem:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate

    def read_serial(self):
        # Simulate receiving a response
        response = f"Received data: {random.randint(0, 100)}"
        print("Receiving:", response)
        return response

    def write_serial(self, message):
        # Simulate writing the message to the serial port
        print(f"Writing: {message}")
        
    def close_serial(self):
        # Simulate closing the serial connection
        print("Serial connection closed")

