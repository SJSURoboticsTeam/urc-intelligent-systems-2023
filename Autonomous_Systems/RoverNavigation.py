from math import atan2, sqrt, pi
import time
import numpy as np
import scipy.linalg as la
import serial
import requests
from queue import PriorityQueue
from Autonomous_Systems.GPS_NAV import GPS_Nav

class KalmanFilter:
    def __init__(self, dt):
        self.dt = dt
        self.A = np.array([[1, 0, dt, 0],
                           [0, 1, 0, dt],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])
        self.C = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])
        self.Q = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 0.1, 0],
                           [0, 0, 0, 0.1]])
        self.R = np.array([[10, 0],
                           [0, 10]])
        self.P = np.eye(self.A.shape[0]) * 1000

    def predict(self, x):
        x = np.dot(self.A, x)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return x

    def update(self, x, z):
        y = z - np.dot(self.C, x)
        S = np.dot(np.dot(self.C, self.P), self.C.T) + self.R
        K = np.dot(np.dot(self.P, self.C.T), la.inv(S))
        x = x + np.dot(K, y)
        self.P = np.dot((np.eye(self.P.shape[0]) - np.dot(K, self.C)), self.P)
        return x

class GridMap:
    def __init__(self, width, height, resolution):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.map = np.zeros((width, height), dtype=int)

    def update(self, x, y):
        i = int(x / self.resolution)
        j = int(y / self.resolution)
        self.map[i, j] += 1

    def get_map(self):
        return self.map

class RoverNavigation:
    def __init__(self, gps_port, imu):
        self.gps = gpsRead(gps_port, 9600)
        self.imu = imu
        self.filter = KalmanFilter(dt=0.1)
        self.position = np.array([0, 0, 0, 0])  # x, y, vx, vy
        self.map = GridMap(width=100, height=100, resolution=0.5)
        self.path = []
        self.rover_nav = GPS_Nav(max_speed=10, max_steering=30, GPS=self.gps, IMU=self.imu, GPS_coordinate_map=[(37.7749, -122.4194)])

    def update_position(self):
        # Get GPS and IMU data
        gps_data = self.gps.get_position()
        imu_data = self.imu.get_data()

        # Update position using Kalman filter
        z = np.array([[gps_data[0]], [gps_data[1]]])
        self.position = self.filter.predict(self.position)
        self.position = self.filter.update(self.position, z)

        # Update map using SLAM
        self.map.update(self.position[0], self.position[1])

        # Plan path using A* algorithm
        start = (int(self.position[0] / self.map.resolution), int(self.position[1] / self.map.resolution))

        # Check if path has already been planned
        if len(self.path) == 0:
            # Plan path to a random unexplored cell
            unexplored = np.argwhere(self.map.get_map() == 0)
            if len(unexplored) > 0:
                goal = tuple(unexplored[np.random.choice(len(unexplored))])
                self.path = self.plan_path(start, goal)
        else:
            # Check if current position is close enough to current path point
            goal = tuple(self.path[-1])
            dx = (self.position[0] - goal[0] * self.map.resolution) ** 2
            dy = (self.position[1] - goal[1] * self.map.resolution) ** 2
            if sqrt(dx + dy) < 0.5:
                self.path.pop()
            else:
                # Continue following current path
                goal = tuple(self.path[-1])

        # Send commands to Rover to follow path
        self.rover_nav.get_steering(gps_data, self.rover_nav.GPS_target)


    def plan_path(self, start, goal):
        # Compute path using A* algorithm
        def heuristic(a, b):
            return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

        def astar(array, start, goal):
            neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

            close_set = set()
            came_from = {}
            gscore = {start: 0}
            fscore = {start: heuristic(start, goal)}
            oheap = []

            heapq.heappush(oheap, (fscore[start], start))

            while oheap:
                current = heapq.heappop(oheap)[1]

                if current == goal:
                    data = []
                    while current in came_from:
                        data.append(current)
                        current = came_from[current]
                    return data[::-1]

                close_set.add(current)
                for i, j in neighbors:
                    neighbor = current[0] + i, current[1] + j
                    tentative_g_score = gscore[current] + heuristic(current, neighbor)
                    if 0 <= neighbor[0] < array.shape[0]:
                        if 0 <= neighbor[1] < array.shape[1]:
                            if array[neighbor[0]][neighbor[1]] == 1:
                                continue
                        else:
                            continue
                    else:
                        continue

                    if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                        continue

                    if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                        came_from[neighbor] = current
                        gscore[neighbor] = tentative_g_score
                        fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                        heapq.heappush(oheap, (fscore[neighbor], neighbor))

            return []

        path = astar(self.map.get_map(), start, goal)

        return path

    def follow_path(self, goal):
        # Compute heading to goal
        dx = goal[0] * self.map.resolution - self.position[0]
        dy = goal[1] * self.map.resolution - self.position[1]
        heading = atan2(dy, dx) * 180.0 / pi
        if heading > 180.0:
            heading -= 360.0
        elif heading < -180.0:
            heading += 360.0
        # Compute distance to goal
        dist = sqrt(dx ** 2 + dy ** 2)

        # Compute forward and angular velocities
        k_v = 0.1  # Velocity gain
        k_w = 0.5  # Heading gain
        v = k_v * dist
        w = k_w * (heading - self.imu.get_heading())

        # Send commands to Rover to follow path
        self.rover_nav.get_steering(self.gps.get_position(), goal)
        self.rover_nav.PID_steer(self.rover_nav.steering, self.rover_nav.steering_target)
        self.rover_nav.set_speed(v)
        self.rover_nav.set_steering(w)
        
    def distance(self, a, b):
        a = np.array(a)
        b = np.array(b)
        return np.linalg.norm(a - b)

    def generate_trajectory(self, start, goal, map):
        queue = PriorityQueue()
        queue.put((0, start))
        visited = {tuple(start): None}
        g_score = {tuple(start): 0}
        max_queue_size = 9
        
        while not queue.empty():
            if queue.qsize() >= max_queue_size:
                break  # queue has grown too large, algorithm failed
            print("Queue:", queue.queue) # Print the current state of the priority queue
            current = queue.get()[1]
            print("Current:", current) # Print the current node being evaluated
            if np.allclose(current, goal):
                break
            
            for direction in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1)]:
                successor = (current[0] + direction[0], current[1] + direction[1])
                if not map.is_traversable(successor) or successor in visited:
                    continue
                
                tentative_g_score = g_score[tuple(current)] + self.distance(current, successor)
                if tuple(successor) not in g_score or tentative_g_score < g_score[tuple(successor)]:
                    g_score[tuple(successor)] = tentative_g_score
                    f_score = tentative_g_score + self.distance(successor, goal)
                    queue.put((f_score, successor))
                    visited[tuple(successor)] = tuple(current)
                    
        current = tuple(goal)
        trajectory = [np.array(current)]
        while current != tuple(start):
            if tuple(current) in visited:
             current = visited[current]
            else:
                break
            trajectory.append(np.array(current))
        trajectory.reverse()
        trajectory = np.array(trajectory)
        
        return trajectory
        
