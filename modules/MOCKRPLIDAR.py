import random
import numpy as np

class MockLidar:
    def __init__(self, port_name='/dev/ttyUSB2', threshold=500):
        self.port_name = port_name
        self.threshold = threshold

    def find_obstacles(self, scan):
        obstacles = []
        for angle in range(0, 360, 10):  # Simulating data for 360 degrees with 10-degree interval
            distance = random.uniform(0, self.threshold)  # Simulate distances within the threshold
            angle_radians = np.radians(angle)
            x = distance * np.cos(angle_radians)
            y = distance * np.sin(angle_radians)
            obstacles.append((x, y, 0))
        return obstacles

    def get_obstacles(self):
        # Simulate scan data
        obstacles = self.find_obstacles([])
        return obstacles

    def closest_obstacle(self, obstacles, position):
        closest_distance = float("inf")
        closest_obstacle = None
        for obstacle in obstacles:
            distance_to_obstacle = np.linalg.norm(np.array(obstacle) - np.array(position))
            if distance_to_obstacle < closest_distance:
                closest_distance = distance_to_obstacle
                closest_obstacle = obstacle
        return closest_obstacle

