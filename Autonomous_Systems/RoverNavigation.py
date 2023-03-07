import numpy as np
import cv2
from scipy.spatial.distance import cdist
import heapq

class RoverNavigation:
    def __init__(self, map_size, map_resolution, max_obstacle_distance):
        self.map_size = map_size
        self.map_resolution = map_resolution
        self.max_obstacle_distance = max_obstacle_distance

        self.x = np.zeros(6)
        self.P = np.eye(6)

        self.F = np.eye(6)
        self.Q = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        self.H = np.zeros((6, 6))
        self.H[0:3, 0:3] = np.eye(3)
        self.R = np.diag([0.1**2]*3 + [1**2]*3)

        self.map = np.zeros(self.map_size, dtype=np.uint8)
        self.obstacles = []

    def update(self, accel_meas, gyro_meas, mag_meas, gps_meas, image):
        # Time update (predict state and covariance)
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

        # Measurement update (correct state and covariance)
        z = np.hstack((accel_meas, gyro_meas, mag_meas))
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

        # Update state transition model based on gyro measurements
        phi, theta, psi = self.x[3], self.x[4], self.x[5]
        self.F[0:3, 3:6] = np.array([[1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
                                     [0, np.cos(phi), -np.sin(phi)],
                                     [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]])

        # Update map and obstacles based on image and GPS measurements
        self.update_map(gps_meas, image)
        self.update_obstacles()

    def update_map(self, gps_meas, image):
        # Convert GPS measurement to map coordinates
        x_map, y_map = self.gps_to_map(gps_meas)

        # Update map based on image
        mask = cv2.inRange(image, (200, 200, 200), (255, 255, 255))
        self.map[y_map:y_map+mask.shape[0], x_map:x_map+mask.shape[1]][mask>0] = 255

        # Update map size based on GPS coordinates of image corners
        x_gps = [gps_meas[1] - mask.shape[1]/(2*self.map_resolution),
                gps_meas[1] + mask.shape[1]/(2*self.map_resolution)]
        y_gps = [gps_meas[0] - mask.shape[0]/(2*self.map_resolution),
                gps_meas[0] + mask.shape[0]/(2*self.map_resolution)]
        x_map, y_map = self.gps_to_map((y_gps[1], x_gps[1]))
        self.map_size = (y_map, x_map)


    def update_obstacles(self):
        # Compute distance transform of map
        dist_map = cv2.distanceTransform(self.map, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)

        # Update maximum obstacle distance based on map size
        self.max_obstacle_distance = max(self.map_size) * self.map_resolution / 2

        # Update obstacles based on distance transform
        self.obstacles = []
        for i in range(self.map.shape[0]):
            for j in range(self.map.shape[1]):
                if dist_map[i, j] > 0 and dist_map[i, j] <= self.max_obstacle_distance:
                    self.obstacles.append((i, j))


    def gps_to_map(self, gps_coords, map_size, map_resolution):
        x_map = int((gps_coords[1] - map_size[1]/2) / map_resolution)
        y_map = int((gps_coords[0] - map_size[0]/2) / map_resolution)
        return (x_map, y_map)

    def map_to_gps(self, map_coords, map_size, map_resolution):
        x_gps = map_coords[1] * map_resolution + map_size[1]/2
        y_gps = map_coords[0] * map_resolution + map_size[0]/2
        return (x_gps, y_gps)

    def plan_path(self, accel_meas, gyro_meas, mag_meas, gps_meas, image, start_gps, goal_gps, map_size, map_resolution, max_obstacle_distance):
        # Update state using IMU measurements
        self.update(accel_meas, gyro_meas, mag_meas, gps_meas, image)

        # Convert start and goal GPS coordinates to map coordinates
        start_map = self.gps_to_map(start_gps, map_size, map_resolution)
        goal_map = self.gps_to_map(goal_gps, map_size, map_resolution)

        # Update map and obstacles based on image and GPS measurements
        map = np.zeros(map_size, dtype=np.uint8)
        mask = cv2.inRange(image, (200, 200, 200), (255, 255, 255))
        map[start_map[1]:start_map[1]+mask.shape[0], start_map[0]:start_map[0]+mask.shape[1]][mask>0] = 255
        dist_map = cv2.distanceTransform(map, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
        obstacles = []
        for i in range(map.shape[0]):
            for j in range(map.shape[1]):
                if dist_map[i, j] > 0 and dist_map[i, j] <= max_obstacle_distance:
                    obstacles.append((i, j))

        # Use A* algorithm to find shortest path between start and goal
        start_node = (start_map[1], start_map[0])
        goal_node = (goal_map[1], goal_map[0])
        path = self.a_star(start_node, goal_node, map, obstacles)

        # Convert path from map coordinates to GPS coordinates
        path_gps = []
        for node in path:
            x_map, y_map = node
            x_gps, y_gps = self.map_to_gps((x_map, y_map), map_size, map_resolution)
            path_gps.append((x_gps, y_gps))

        return path_gps




    def a_star(self, start, goal, map, obstacles):
        open_list = []
        closed_list = set()
        heapq.heappush(open_list, (0, start))
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        parent = {start: None}

        while open_list:
            current = heapq.heappop(open_list)[1]
            if current == goal:
                return self.reconstruct_path(parent, current)
            closed_list.add(current)
            for neighbor in self.get_neighbors(current, map):
                if neighbor in closed_list or neighbor in obstacles:
                    continue
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    parent[neighbor] = current
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))

        return []

    def get_neighbors(self, node, map):
        neighbors = []
        for i in range(-1, 2):
            for j in range(-1, 2):
                x = node[0] + i
                y = node[1] + j
                if (i == 0 and j == 0) or x < 0 or x >= map.shape[0] or y < 0 or y >= map.shape[1]:
                    continue
                neighbors.append((x, y))
        return neighbors

    def reconstruct_path(self, parent, current):
        path = []
        while current is not None:
            path.append(current)
            current = parent[current]
        return path[::-1]
    
    def heuristic(self, node1, node2):
        return cdist([node1], [node2], 'cityblock')[0][0]




# import numpy as np
# import cv2

# from rover_navigation import RoverNavigation


# def main():
#     # Initialize RoverNavigation object
#     nav = RoverNavigation()

#     # Generate some sample measurements and images
#     accel_meas = np.array([0, 0, 9.81])
#     gyro_meas = np.array([0, 0, 0])
#     mag_meas = np.array([0, 0, 0])
#     gps_meas = np.array([42.35, -71.05])
#     image = np.zeros((480, 640, 3), dtype=np.uint8)

#     # Set start and goal GPS coordinates
#     start_gps = (42.3505, -71.0503)
#     goal_gps = (42.3515, -71.0515)

#     # Plan path from start to goal
#     path_gps = nav.plan_path(accel_meas, gyro_meas, mag_meas, gps_meas, image, start_gps, goal_gps)

#     print(f"Planned path: {path_gps}")


# if __name__ == '__main__':
#     main()
