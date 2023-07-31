import random
import requests
import math

class MockGPSRead:
    def __init__(self):
        # No need to open a serial port in the mock class
        pass

    def send_request(self, LonLat):
        # Simulate sending data to the server
        print("Simulated request sent:", LonLat)

    def get_position(self):
        LonLat = {"longitude": 0, "latitude": 0}

        # Simulate generating random GPS coordinates
        latitude = random.uniform(-90, 90)
        longitude = random.uniform(-180, 180)
        LonLat["longitude"] = longitude
        LonLat["latitude"] = latitude

        # Pretend to send the data using send_request() method
        self.send_request(LonLat)

        return [longitude, latitude]

