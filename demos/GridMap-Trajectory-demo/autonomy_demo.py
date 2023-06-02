import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.colors import ListedColormap
from queue import PriorityQueue
from matplotlib.patches import Arrow
import random
import utm


class GridMapSimulator:
    def __init__(self, resolution, map_width, map_height, targets, init_gps, lidar_range=1, num_initial_obstacles=20, obstacle_memory=4, interval=200):
        self.resolution = resolution
        self.map_width = map_width
        self.map_height = map_height
        self.interval = interval
        self.ani = None
        self.path_plot = None  # Add an attribute to store the path plot
        self.map = np.zeros((self.map_height, self.map_width))  # use a numpy array to represent the map
        self.targets = targets
        self.current_target_index = 0
        self.target_x, self.target_y = targets[self.current_target_index]

        self.reached_destination = False
        self.rover_direction = 0

        # Set the initial position based on the provided GPS coordinate
        init_lon, init_lat = init_gps
        self.rover_x, self.rover_y = gps_to_grid_coordinates(init_lat, init_lon, min_utm_x, min_utm_y, max_utm_x, max_utm_y)

        self.lidar_range = lidar_range
        self.generate_initial_obstacles(num_initial_obstacles)
        self.obstacle_memory = obstacle_memory
        self.detect_obstacle()

    def generate_initial_obstacles(self, num_obstacles):
        cur_obstacles = 0
        while cur_obstacles < num_obstacles:
            x = np.random.randint(0, self.map_width)
            y = np.random.randint(0, self.map_height)
            if not self.map[y, x] < -1 and (x, y) not in self.targets:
                cur_obstacles += 1
                self.map[y, x] = -1


    def init_visualization(self):
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Grid Map")
        self.path_line, = self.ax.plot([], [], color='lime')
        self.path_plot, = self.ax.plot([], [], color='red', linestyle='-')

        colors = ['black', 'gray', 'white']
        # now we want bounds so that anything below -1 is black, -1 is gray, and anything above -1 is white
        bounds = [-2 - self.obstacle_memory, -1, 0, 1]
        print(bounds)
        cmap = ListedColormap(colors)
        norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)
        self.ani = FuncAnimation(self.fig, animate, fargs=(self,), interval=self.interval)
        self.grid_img = self.ax.imshow(self.map, origin='lower', cmap=cmap, norm=norm)

        # Add the initial position of the rover as a red dot
        arrow_length = 1
        arrow_width = 2
        self.rover_arrow = Arrow(self.rover_x - arrow_length / 2, self.rover_y - arrow_width / 2, arrow_length, arrow_width, color='gray', zorder=2)
        self.ax.add_patch(self.rover_arrow)

        # Add the destination positions as blue dots and number them
        self.target_dots = []
        for i, (target_x, target_y) in enumerate(self.targets):
            target_dot = self.ax.scatter(target_x, target_y, c='blue')
            self.ax.text(target_x + 0.5, target_y + 0.5, str(i + 1), fontsize=12, color='blue')
            self.target_dots.append(target_dot)



    def update_visualization(self, target_x, target_y):
        # Find the optimal path from the current position to the target position using A*
        path = self.find_path(self.rover_x, self.rover_y, target_x, target_y)
        if path is not None:
            path_x, path_y = zip(*path)
            self.path_line.set_data(path_x, path_y)

        arrow_length = 1
        arrow_width = 1
        self.rover_arrow.remove()

        x_offset, y_offset = 0.5, 0.5
        self.rover_arrow = Arrow(self.rover_x + 0.5 - x_offset,
                                 self.rover_y + 0.5 - y_offset,
                                 arrow_length * np.cos(self.rover_direction),
                                 arrow_length * np.sin(self.rover_direction),
                                 color='gray', zorder=2)

        self.ax.add_patch(self.rover_arrow)

        self.map[self.rover_y, self.rover_x] = 1
        self.grid_img.set_data(self.map)

        # Check if the rover has reached the target position
        if self.rover_x == target_x and self.rover_y == target_y:
            print("I have made it to the destination!")
            # plt.close(self.fig)  # Stop the animation
            # exit(1)

        return [self.grid_img, self.rover_arrow, *self.target_dots, self.path_plot]


    def find_path(self, start_x, start_y, goal_x, goal_y):
        def heuristic(a, b):
            dx = abs(b[0] - a[0])
            dy = abs(b[1] - a[1])
            return (dx + dy) + (np.sqrt(2) - 2) * min(dx, dy)

        frontier = PriorityQueue()
        frontier.put((0, (start_x, start_y)))
        came_from = {}
        cost_so_far = {}
        came_from[(start_x, start_y)] = None
        cost_so_far[(start_x, start_y)] = 0

        while not frontier.empty():
            current = frontier.get()[1]

            if current == (goal_x, goal_y):
                path = [current]
                while current != (start_x, start_y):
                    current = came_from[current]
                    path.append(current)
                return path[::-1]

            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (-1, 1), (1, -1), (-1, -1)]:
                neighbor = (current[0] + dx, current[1] + dy)

                if neighbor[0] < 0 or neighbor[0] >= self.map_width or neighbor[1] < 0 or neighbor[1] >= self.map_height:
                    continue

                if (dx != 0 and dy != 0) and (self.map[current[1], current[0] + dx] < -1 or self.map[current[1] + dy, current[0]] < -1):
                    continue

                if self.map[neighbor[1], neighbor[0]] < -1:
                    continue

                new_cost = cost_so_far[current] + 1
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic((goal_x, goal_y), neighbor)
                    frontier.put((priority, neighbor))
                    came_from[neighbor] = current

        return None


    def detect_obstacle(self):
        # reduce the memory of all the obstacles currently seen (anything with a counter less than -1)
        self.map[self.map < -1] += 1

        # now, simulate checking for obstacles in the 8 directions around the rover
        # this line gets a subarray of the map that is 3x3 (when lidar range is 1, 5x5 for lidar range 2, etc)
        # centered on the rover, using max and min to make sure we don't go out of bounds

        nearby_map = self.map[max(self.rover_y - self.lidar_range, 0):min(self.rover_y +  self.lidar_range + 1, self.map_height),
                                max(self.rover_x - self.lidar_range, 0):min(self.rover_x +  self.lidar_range + 1, self.map_width)]
        nearby_map[nearby_map <= -1] = -1 - self.obstacle_memory

    def expand_map(self, new_width, new_height):
        new_map = np.zeros((new_height, new_width))
        new_map[:self.map_height, :self.map_width] = self.map
        self.map = new_map
        self.map_width = new_width
        self.map_height = new_height


    def move_rover(self):
        if self.reached_destination:
            return
        # Detect obstacles before moving
        self.detect_obstacle()

        target_x, target_y = self.targets[self.current_target_index]

        # Find the optimal path from the current position to the target position using A*
        path = self.find_path(self.rover_x, self.rover_y, target_x, target_y)
        if path is None or len(path) < 2:
            # If there is no path or the path is too short, do not move the rover
            print("No path found or path too short")
            return

        # Move the rover one step along the optimal path
        new_x, new_y = path[1]

        dx, dy = new_x - self.rover_x, new_y - self.rover_y
        new_direction = np.arctan2(dy, dx)
        if new_direction != self.rover_direction:
            self.rover_direction = new_direction
            print(f"Turned to angle {np.degrees(self.rover_direction)}")

        print("Moved to position ({}, {})".format(new_x, new_y))
        self.map[self.rover_y, self.rover_x] = 0  # Clear the old rover's position
        self.rover_x, self.rover_y = new_x, new_y

        # Add the new position to the path plot
        path_x, path_y = self.path_plot.get_data()
        path_x = np.append(path_x, self.rover_x)
        path_y = np.append(path_y, self.rover_y)
        self.path_plot.set_data(path_x, path_y)

        # Check if the rover has reached the target position
        if self.rover_x == target_x and self.rover_y == target_y:
            print("I have made it to the destination!")
            self.reached_destination = True
            if self.current_target_index + 1 < len(self.targets):
                self.current_target_index += 1
                self.reached_destination = False
                print(f"Moving to the next target: {self.targets[self.current_target_index]}")
            else:
                print("All targets reached!")



# Example usage
def animate(frame, grid_map, *args):
    grid_map.move_rover()
    target_x, target_y = grid_map.targets[grid_map.current_target_index]
    return grid_map.update_visualization(target_x, target_y)

def generate_random_targets(num_targets, map_width, map_height):
    random_targets = []
    for _ in range(num_targets):
        x = random.randint(0, map_width - 1)
        y = random.randint(0, map_height - 1)
        random_targets.append((x, y))
    return random_targets


def gps_to_grid_coordinates(lat, lon, min_utm_x, min_utm_y, max_utm_x, max_utm_y):
    utm_x, utm_y, _, _ = utm.from_latlon(lat, lon)
    normalized_x = (utm_x - min_utm_x) / (max_utm_x - min_utm_x)
    normalized_y = (utm_y - min_utm_y) / (max_utm_y - min_utm_y)
    x = int((normalized_x * (map_width - 1)) + 0.5)
    y = int((normalized_y * (map_height - 1)) + 0.5)
    return x, y

# Made from https://www.gpsvisualizer.com/draw/
GPSList = [
    [-121.8819474, 37.3372215],
    [-121.8819252, 37.3371282],
    [-121.8818977, 37.3370621],
    [-121.8818334, 37.3370451],
    [-121.8818421, 37.3371197],
    [-121.8818763, 37.3371991],
    [-121.8818998, 37.3372471],
    [-121.881935, 37.337250],
]

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