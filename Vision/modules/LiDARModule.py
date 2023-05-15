import matplotlib.pyplot as plt
import numpy as np
from rplidar import RPLidar, RPLidarException
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler

class LiDARModule:
    def __init__(self, port_name):
        self.port_name = port_name
        self.lidar = RPLidar(self.port_name)
        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-5000, 5000)
        self.ax.set_ylim(-5000, 5000)
        self.ax.arrow(0, 0, 0, 1000, head_width=100, head_length=200, fc='k', ec='k')
        self.points = self.ax.scatter([], [], s=2, c='red')
        self.cluster_points = self.ax.scatter([], [], s=5, c='blue')
        self.obstacles = []
        
    def run(self):
        position = 0 # current position of device on x-axis
        try:
            for scan in self.lidar.iter_scans():
                try:
                    x = []
                    y = []
                    for (_, angle, distance) in scan:
                        x.append(distance * np.sin(np.radians(angle)))
                        y.append(distance * np.cos(np.radians(angle)))
                    xy = np.array(list(zip(x, y)))
                    xy_scaled = StandardScaler().fit_transform(xy)
                    db = DBSCAN(eps=0.3, min_samples=10).fit(xy_scaled)
                    labels = db.labels_
                    unique_labels = set(labels)
                    colors = [plt.cm.Spectral(each) for each in np.linspace(0, 1, len(unique_labels))]
                    for k, col in zip(unique_labels, colors):
                        if k == -1:
                            col = [0, 0, 0, 1]
                        class_member_mask = (labels == k)
                        xy_class = xy[class_member_mask]
                        self.cluster_points.set_offsets(xy_class)
                        self.obstacles.append(xy_class)
                    obstacle_coords = self.get_obstacle_coordinates(position)
                    for output in obstacle_coords:
                        yield output
                    self.points.set_offsets(np.c_[x, y])
                    self.fig.canvas.draw()
                    plt.pause(0.001)
                except RPLidarException as e:
                    if str(e) == 'Incorrect descriptor starting bytes':
                        continue
                    if str(e) == 'Wrong body size':
                        continue
                    else:
                        continue

        except KeyboardInterrupt:
            print('Stopping.')
            self.lidar.stop()
            self.lidar.disconnect()
        except RPLidarException as e:
            print(e)
            self.run()
    
    def get_obstacle_coordinates(self, position):
        obstacle_coords = []
        for obstacle in self.obstacles:
            obstacle_x = obstacle[:, 0]
            if np.max(obstacle_x) < position:
                obstacle_coords.append(("Left", obstacle))
            elif np.min(obstacle_x) > position:
                obstacle_coords.append(("Right", obstacle))
            else:
                obstacle_coords.append(("Center", obstacle))
        return obstacle_coords

    def stop(self):
        self.lidar.stop()
        self.lidar.disconnect()


