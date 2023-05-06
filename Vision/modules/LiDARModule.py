import matplotlib.pyplot as plt
import numpy as np
from rplidar import RPLidar, RPLidarException

class LiDARModule:
    def __init__(self, port_name):
        self.lidar = RPLidar(port_name)
        self.fig, self.ax = plt.subplots()
        # Set up the plot parameters
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-5000, 5000)
        self.ax.set_ylim(-5000, 5000)
        # Set interactive mode on
        plt.ion()
        # Create scatter plot
        self.points = self.ax.scatter([], [], s=2, c='red')

    def start_device(self):
        try:
            for scan in self.lidar.iter_scans():
                try:
                    # Get the x and y positions of the points
                    x = []
                    y = []
                    for (_, angle, distance) in scan:
                        # Convert the polar coordinates to cartesian coordinates
                        x.append(distance * np.sin(np.radians(angle)))  # Flip x and y
                        y.append(distance * np.cos(np.radians(angle)))  # Flip x and y

                    # Add the new points to the scatter plot
                    self.points.set_offsets(np.c_[x, y])

                    # Redraw the plot
                    self.fig.canvas.draw()

                    # Pause to allow plot to update
                    plt.pause(0.001)

                except RPLidarException as e:
                    # If an incorrect descriptor starting byte exception is caught,
                    # print the error message and continue with the next iteration
                    if str(e) == 'Incorrect descriptor starting bytes':
                        print('Caught Incorrect descriptor starting bytes exception')
                        continue
                    else:
                        raise e

        except KeyboardInterrupt:
            print('Stopping.')
            self.lidar.stop()
            self.lidar.disconnect()


    def stop_device(self):
        self.lidar.stop()
        self.lidar.disconnect()