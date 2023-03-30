import sys

sys.path.append('../../')
import numpy as np
import time


class GridMap:
    def __init__(self, resolution):
        self.resolution = resolution
        self.map = {}

    def update(self, x, y):
        i = int(x / self.resolution)
        j = int(y / self.resolution)
        if (i, j) not in self.map:
            self.map[(i, j)] = 0
        self.map[(i, j)] += 1

    def update_obstacle(self, x, y):
        i = int(x / self.resolution)
        j = int(y / self.resolution)
        self.map[(i, j)] = -1  # set the grid cell to -1 to represent an obstacle

    def update_obstacles(self, lidar):
        obstacles = lidar.get_obstacles()
        for obstacle in obstacles:
            self.update_obstacle(obstacle[0], obstacle[1])

    def get_map(self):
        return self.map




# class GridMap:
#     def __init__(self, width, height, resolution):
#         self.width = width
#         self.height = height
#         self.resolution = resolution
#         self.map = np.zeros((width, height), dtype=int)

#     def update(self, x, y):
#         i = int(x / self.resolution)
#         j = int(y / self.resolution)
#         if i >= 0 and i < self.width and j >= 0 and j < self.height:
#             self.map[i, j] += 1

#     def get_map(self):
#         return self.map