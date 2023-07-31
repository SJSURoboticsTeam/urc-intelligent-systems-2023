import random

class MockIR_Sensor:
    def __init__(self, pin):
        self.pin = pin

    def DetectObject(self):
        # Simulate object detection by generating a random value (0 or 1)
        # You can modify this logic to mimic specific scenarios for testing
        return random.randint(0, 1)

