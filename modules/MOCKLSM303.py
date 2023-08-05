import random
import math

class MockCompass:
    def __init__(self):
        self.heading_range = (0.0, 360.0)

    def get_random_value(self, value_range):
        return random.uniform(value_range[0], value_range[1])

    def get_heading(self):
        """Get random compass heading in degrees."""
        heading = self.get_random_value(self.heading_range)
        return round(heading, 3)

