from math import floor, pi
from rplidar import RPLidar
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from matplotlib.axes import Axes
from matplotlib.collections import LineCollection
from threading import Thread
import time

#==================================
# Initial connect and set up Lidar
#----------------------------------
PORT_NAME = 'COM10'
lidar = None
lidar_iter = None
while True:
    try: # Try and try again until Lambs become lions and 
        lidar = RPLidar(PORT_NAME)
        lidar_iter = iter(lidar.iter_scans(max_buf_meas=10000))
        next(lidar_iter) # and until the Lidar is able to yield stuff without errors
    except Exception as e:
        lidar.disconnect()
        print(e)
        continue
    break
#====================================

#====================================
# Running the Lidar
#------------------------------------
scan_data = [0]*360 # buffer to hold distance data
def look(verbose=True):
    """Fetch new distance data and update the buffer"""
    global scan_data
    scan = list(reversed(next(lidar_iter)))
    for (_, angle, distance) in scan:
        scan_data[floor(angle)%360] = distance
def spin(stop_check, verbose=True):
    """Keep fetching and updating buffer with distance data"""
    while not stop_check():
        look()
stop_spinning=False
t = Thread(target=spin, args=(lambda: stop_spinning, ))
t.start()
#====================================


try:
    fig, axs = plt.subplots(ncols=2, subplot_kw={'projection': 'polar'})
    axl: Axes = axs[0]
    # axl.axis('off')
    axl.set_title('Upside_down_lidar')
    axl.set_rmax(1)
    # axl.set_rmin(0)
    scatL = axl.scatter([], [], marker='o', c='orange')
    obs_collectionL = LineCollection([])
    axl.add_collection(obs_collectionL)
    axl.set_visible(False)

    axr: Axes = axs[1]
    # axr.axis('off')
    axr.set_title('Upside_up_lidar')
    axr.set_rmax(1) # All distances will be scaled to a 0-1 range
    # axr.set_rmin(0)
    scatR = axr.scatter([], [], marker='o', c='orange')
    obs_collectionR = LineCollection([])
    axr.add_collection(obs_collectionR)
    def join_near_points(points, flipped = False, thresh=0.2):
        """Given points in polar coordinates,
        Returns a 2D List of obstacles, 
        where each obstacle is a collection of points near each other"""
        obstacles = [[(0,points[0])]]
        for i, p in enumerate(points[1:]):
            if abs(p-obstacles[-1][-1][1]) > thresh: # If point p is thresh far away from the last point in the last obstacle, then it is part of a new obstacle
                obstacles.append([])
            angle  = 2*pi/360*(i+1) if not flipped else 2*pi/360*(360-i+1)
            obstacles[-1].append((angle,p))
        # check if first and last obstacles are actually the same obstacles
        if (obstacles[-1][-1][1]-obstacles[0][0][1]):
            obstacles[0] = obstacles[-1] + obstacles[0]
            obstacles.pop()
        print()
        print(obstacles)
        print()
        return obstacles
    def update(_):
        modded = []
        # scan_data_rev = list(reversed(scan_data))
        # RMAX=max(scan_data)*1.2
        # axr.set_rmax(RMAX)
        # scatR.set_offsets([(2*pi/360*(359-i), scan_data[i]) for i in range(0,360)])
        # modded.append(scatR)
        maxR = max(scan_data)*1.1
        scaled_data = [i/maxR for i in scan_data]
        obs_collectionR.set_segments(join_near_points(scaled_data, True))
        modded.append(obs_collectionR)

        # axl.set_rmax(RMAX)
        # scatL.set_offsets([(2*pi/360*i, scan_data[i]) for i in range(0,360)])
        # modded.append(scatL)

        # obs_collectionL.set_segments(join_near_points(scaled_data))
        # modded.append(obs_collectionL)

        return modded

    anime = anim.FuncAnimation(
        fig,
        update,
        1,
        interval=1,
        blit=True
    )
    plt.show()
except Exception as e:
    print(e)





stop_spinning=True
t.join()
lidar.stop()
lidar.stop_motor()
lidar.disconnect()