import matplotlib.pyplot as plt
import numpy as np
from rplidar import RPLidar, RPLidarException

PORT_NAME = '/dev/tty.usbserial-0001'

lidar = RPLidar(PORT_NAME)

fig, ax = plt.subplots()

# Set up the plot parameters
ax.set_aspect('equal')
ax.set_xlim(-5000, 5000)
ax.set_ylim(-5000, 5000)

# Set interactive mode on
plt.ion()

# Create scatter plot
points = ax.scatter([], [], s=2, c='red')

try:
    for scan in lidar.iter_scans():
        try:
            # Get the x and y positions of the points
            x = []
            y = []
            for (_, angle, distance) in scan:
                # Convert the polar coordinates to cartesian coordinates
                x.append(distance * np.sin(np.radians(angle)))  # Flip x and y
                y.append(distance * np.cos(np.radians(angle)))  # Flip x and y

            # Add the new points to the scatter plot
            points.set_offsets(np.c_[x, y])

            # Redraw the plot
            fig.canvas.draw()

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
    lidar.stop()
    lidar.disconnect()

# Turn interactive mode off and show plot
plt.ioff()
plt.show()
