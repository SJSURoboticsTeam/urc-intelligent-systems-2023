from rplidar import RPLidar
import numpy as np
import matplotlib.pyplot as plt
from rplidar import RPLidarException


class Lidar:
    def __init__(self, port_name='/dev/ttyUSB2', threshold=500):
        self.port_name = port_name
        self.threshold = threshold
        self.lidar = None

    def find_obstacles(self, scan):
            obstacles = []
            for (_, angle, distance) in scan:
                if distance <= self.threshold:
                    angle_radians = np.radians(angle)
                    x = distance * np.cos(angle_radians)
                    y = distance * np.sin(angle_radians)
                    obstacles.append((x, y, 0))
            return obstacles if obstacles else None


    def get_obstacles(self):
        self.lidar = RPLidar(self.port_name)
        scan = self.lidar.iter_scans()
        try:
            obstacles = self.find_obstacles(next(scan))
        except RPLidarException:
            obstacles = None
        self.lidar.stop()
        self.lidar.disconnect()
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
