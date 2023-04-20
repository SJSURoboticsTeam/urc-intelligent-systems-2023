import sys

sys.path.append('../../')
import numpy as np
import time
import utm


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
    
    def gps_to_grid_coordinates(lat, lon, min_utm_x, min_utm_y, max_utm_x, max_utm_y):
        utm_x, utm_y, _, _ = utm.from_latlon(lat, lon)
        normalized_x = (utm_x - min_utm_x) / (max_utm_x - min_utm_x)
        normalized_y = (utm_y - min_utm_y) / (max_utm_y - min_utm_y)
        x = int((normalized_x * (map_width - 1)) + 0.5)
        y = int((normalized_y * (map_height - 1)) + 0.5)
        return x, y
    

    def run_gridmap(self):
        map_width = 25
        map_height = 25
        coordinate_list = []

        # Convert GPS to UTM and find the min and max UTM coordinates
        utm_coords = [utm.from_latlon(lat, lon)[:2] for lon, lat in GPSList]
        min_utm_x, min_utm_y = map(min, zip(*utm_coords))
        max_utm_x, max_utm_y = map(max, zip(*utm_coords))

        for i in range(len(GPSList)):
            lon, lat = GPSList[i]
            x, y = gps_to_grid_coordinates(lat, lon, min_utm_x, min_utm_y, max_utm_x, max_utm_y)
            coordinate_list.append((x, y))
            print("grid point", x, y)




        resolution = 1
        lidar_range = 1 # this is how many squares away the rover can see an obstacle
        map_width = 30
        map_height = 30
        initial_obstacles = 100
        obstacle_memory = 12 # this is the number of frames that an obstacle is remembered/included in the astar search after it was detected.
        animation_speed = 100
        num_targets = 3

        init_gps = [-121.881935, 37.337250]
        grid_map = GridMapSimulator(resolution, map_width, map_height, coordinate_list, init_gps,
                                    lidar_range=lidar_range,
                                    num_initial_obstacles=initial_obstacles,
                                    obstacle_memory=obstacle_memory,
                                    interval=animation_speed)
        target_x, target_y = coordinate_list[grid_map.current_target_index]
        grid_map.init_visualization()
        plt.show()




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