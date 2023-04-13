import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.colors import ListedColormap
from queue import PriorityQueue
from matplotlib.patches import Arrow
import random
import utm
import csv


class GridMapSimulator:
    def __init__(self, resolution, map_width, map_height, targets, init_gps, num_initial_obstacles=20, interval=200):
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
        self.obstacles = self.generate_initial_obstacles(num_initial_obstacles)
        self.reached_destination = False
        self.rover_direction = 0

        # Set the initial position based on the provided GPS coordinate
        init_lon, init_lat = init_gps
        self.rover_x, self.rover_y = gps_to_grid_coordinates(init_lat, init_lon, min_utm_x, min_utm_y, max_utm_x, max_utm_y)

    def generate_initial_obstacles(self, num_obstacles):
        obstacles = []
        while len(obstacles) < num_obstacles:
            x = np.random.randint(0, self.map_width)
            y = np.random.randint(0, self.map_height)
            if self.map[y, x] != -1 and (x, y) != (self.target_x, self.target_y):
                obstacles.append((x, y))
                self.map[y, x] = -1
        return obstacles


    def init_visualization(self):
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Grid Map")
        self.path_line, = self.ax.plot([], [], color='lime')
        self.path_plot, = self.ax.plot([], [], color='red', linestyle='-')
        colors = ['black', 'white']
        cmap = ListedColormap(colors)
        self.ani = FuncAnimation(self.fig, animate, fargs=(self,), interval=self.interval)
        self.grid_img = self.ax.imshow(self.map, origin='lower', cmap=cmap)

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

                if (dx != 0 and dy != 0) and (self.map[current[1], current[0] + dx] == -1 or self.map[current[1] + dy, current[0]] == -1):
                    continue

                if self.map[neighbor[1], neighbor[0]] == -1:
                    continue

                new_cost = cost_so_far[current] + 1
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic((goal_x, goal_y), neighbor)
                    frontier.put((priority, neighbor))
                    came_from[neighbor] = current

        return None


    def detect_obstacle(self):
        if self.reached_destination:
            return None
        # Simulate the detection of an obstacle in the vicinity of the rover
        # For simplicity, we randomly generate an obstacle within a square region around the rover
        region_size = 3
        x = np.random.randint(self.rover_x - region_size, self.rover_x + region_size + 1)
        y = np.random.randint(self.rover_y - region_size, self.rover_y + region_size + 1)

        if x < 0 or x >= self.map_width or y < 0 or y >= self.map_height:
            return

        # Check if the randomly generated coordinates are part of the current path
        path_x, path_y = self.path_plot.get_data()
        if (x, y) in zip(path_x, path_y):
            return None

        if self.map[y, x] != -1 and (x, y) != (self.target_x, self.target_y):
            self.map[y, x] = -1
            print(f"Obstacle detected at ({x}, {y})")
            return x, y
        else:
            return None


    def move_rover(self):
        if self.reached_destination:
            return
        # Detect obstacles before moving
        detected_obstacle = self.detect_obstacle()
        if detected_obstacle:
            self.obstacles.append(detected_obstacle)

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
    grid_map.detect_obstacle()
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


GPSList = []
red = "#ff0000" # 
green = "#00ff00" 
with open('gpsloc.txt','r') as gps_locs:
    reader = csv.reader(gps_locs, delimiter='\t')
    for row in reader: 
        if row[0] is "type" :
            next
        lat = row[1]
        long = row[2]
        color = row[4]
        if color == green:
            GPSList.append([float(long),float(lat)])

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
map_width = 30
map_height = 30
# target_x, target_y = 30, 5
initial_obstacles = 1
animation_speed = 100
num_targets = 3

random_targets = generate_random_targets(num_targets, map_width, map_height)
print("Random targets:", random_targets)

init_gps = [-121.88177050000002, 37.336928833333324]
grid_map = GridMapSimulator(resolution, map_width, map_height, coordinate_list, init_gps, num_initial_obstacles=initial_obstacles, interval=animation_speed)
target_x, target_y = coordinate_list[grid_map.current_target_index]
grid_map.init_visualization()
plt.show()