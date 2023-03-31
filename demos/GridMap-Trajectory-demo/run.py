import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.colors import ListedColormap
from queue import PriorityQueue

class GridMap:
    def __init__(self, resolution, map_width, map_height, num_initial_obstacles=20):
        self.resolution = resolution
        self.map_width = map_width
        self.map_height = map_height
        self.rover_x = 0
        self.rover_y = 0
        self.map = np.zeros((self.map_height, self.map_width))  # use a numpy array to represent the map
        self.obstacles = self.generate_initial_obstacles(num_initial_obstacles)

    def generate_initial_obstacles(self, num_obstacles):
        obstacles = []
        while len(obstacles) < num_obstacles:
            x = np.random.randint(0, self.map_width)
            y = np.random.randint(0, self.map_height)
            if self.map[y, x] != -1:
                obstacles.append((x, y))
                self.map[y, x] = -1
        return obstacles

    def init_visualization(self):
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Grid Map")
        self.path_line, = self.ax.plot([], [], color='green')
        colors = ['black', 'white']  # remove the red color from the colormap
        cmap = ListedColormap(colors)
        self.grid_img = self.ax.imshow(self.map, origin='lower', cmap=cmap)

        # add the initial position of the rover as a red dot
        self.rover_dot = self.ax.scatter(self.rover_x, self.rover_y, c='red')

        # add the destination position as a green dot
        self.target_dot = self.ax.scatter(target_x, target_y, c='blue')

    def update_visualization(self, target_x, target_y):
        # Find the optimal path from the current position to the target position using A*
        path = self.find_path(self.rover_x, self.rover_y, target_x, target_y)
        if path is not None:
            path_x, path_y = zip(*path)
            self.path_line.set_data(path_x, path_y)

        self.rover_dot.set_offsets(np.array([[self.rover_x, self.rover_y]]))

        self.map[self.rover_y, self.rover_x] = 1
        self.grid_img.set_data(self.map)

        # Check if the rover has reached the target position
        if self.rover_x == target_x and self.rover_y == target_y:
            print("I have made it!")
            plt.close(self.fig)  # Stop the animation

        return [self.grid_img, self.rover_dot, self.target_dot]


    def find_path(self, start_x, start_y, goal_x, goal_y):
        # Define the A* heuristic function (diagonal distance)
        def heuristic(a, b):
            dx = abs(b[0] - a[0])
            dy = abs(b[1] - a[1])
            return (dx + dy) + (np.sqrt(2) - 2) * min(dx, dy)

        # Initialize the A* algorithm
        frontier = PriorityQueue()
        frontier.put((0, (start_x, start_y)))
        came_from = {}
        cost_so_far = {}
        came_from[(start_x, start_y)] = None
        cost_so_far[(start_x, start_y)] = 0

        # Run the A* algorithm
        while not frontier.empty():
            current = frontier.get()[1]

            if current == (goal_x, goal_y):
                # If the goal has been reached, reconstruct the path and return it
                path = [current]
                while current != (start_x, start_y):
                    current = came_from[current]
                    path.append(current)
                return path[::-1]

            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (-1, 1), (1, -1), (-1, -1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                if neighbor[0] < 0 or neighbor[0] >= self.map_width or neighbor[1] < 0 or neighbor[1] >= self.map_height or self.map[neighbor[1], neighbor[0]] == -1:
                    continue
                new_cost = cost_so_far[current] + 1
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic((goal_x, goal_y), neighbor)
                    frontier.put((priority, neighbor))
                    came_from[neighbor] = current

        # If the goal could not be reached, return None
        return None




    def move_rover(self, target_x, target_y):
            # Find the direction towards the target position
            print("Current position: ({}, {})".format(self.rover_x, self.rover_y))
            print("Target position: ({}, {})".format(target_x, target_y))
            dx = int(target_x - self.rover_x)
            dy = int(target_y - self.rover_y)

            # Calculate the Euclidean distance to the target position
            dist = np.sqrt(dx**2 + dy**2)

            # Normalize the direction vector if it is non-zero
            if dist > 0:
                dx = dx / dist
                dy = dy / dist

            # Move the rover towards the target position, avoiding obstacles
            new_x, new_y = int(self.rover_x + dx), int(self.rover_y + dy)
            if np.isnan(new_x) or np.isnan(new_y):
                # If the new position is not a number, move randomly
                new_x = np.random.randint(0, self.map_width)
                new_y = np.random.randint(0, self.map_height)
                print("Moved randomly to position ({}, {})".format(new_x, new_y))
            elif new_x < 0 or new_x >= self.map_width or new_y < 0 or new_y >= self.map_height:
                # If the new position is out of bounds, move randomly
                new_x = np.random.randint(0, self.map_width)
                new_y = np.random.randint(0, self.map_height)
                print("Moved randomly to position ({}, {})".format(new_x, new_y))
            elif self.map[new_y, new_x] == -1:
                # If the new position is occupied by an obstacle, move randomly
                new_x = np.random.randint(0, self.map_width)
                new_y = np.random.randint(0, self.map_height)
                print("Moved randomly to position ({}, {})".format(new_x, new_y))
            else:
                print("Moved to position ({}, {})".format(new_x, new_y))
                self.map[self.rover_y, self.rover_x] = 0  # Clear the old rover's position
                self.rover_x, self.rover_y = new_x, new_y


# Example usage

def animate(frame, grid_map, target_x, target_y):
    grid_map.move_rover(target_x, target_y)
    return grid_map.update_visualization(target_x, target_y)

resolution = 1
map_width = 20
map_height = 20
target_x, target_y = 15, 15
grid_map = GridMap(resolution, map_width, map_height, num_initial_obstacles=20)
grid_map.init_visualization()
ani = FuncAnimation(grid_map.fig, animate, fargs=(grid_map, target_x, target_y), interval=100)

plt.show()
