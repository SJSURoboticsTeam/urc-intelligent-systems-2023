"""
Approach to running the Lidar set up.
I'm thinking of running a separate thread for each and every task being run for the Lidar navigation.
Here are the tasks that will each get its own thread.
    1. Lidar Data Consumption:
        The Lidar produces a lot of sensor data. This data needs to be consumed quickly 
        or else it will build up inside the Lidar's internal buffer. This build up creates
        a lag in the delivery of Lidar data. To prevent this there will be a dedicated thread
        to consume data from the Lidar and make it available to other threads.
    2. Visualization:
        I'm a visual person and visualization helps to understand whats going on in code. So I'm
        making this thread to visualize all code activity. This thread will visualize what the 
        Lidar sees, it will visualize the navigation planning and anything else that will be helpful
        to visualize. Important! - This should not be where processing should happen. When visualization 
        is not needed it should be possible to just stop this thread and everything else should run normally
    3.  Data maintenance:
        A separate thread is to process Lidar data into point cloud with persistance and to convert that
        into obstacles

"""

from math import floor, pi, sqrt, cos
from rplidar import RPLidar
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from matplotlib.axes import Axes
from matplotlib.collections import LineCollection, PathCollection
from threading import Thread, Lock
import time
import traceback
import numpy as np

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
scan_data = list() # buffer to hold distance data # use same ters scan/measures/
def fill_measures(distances):
    "Sets and array of (signal quality, angle (rad), distance (m))"
    global scan_data
    scan_data = distances
def get_measures(flipped=True):
    "Gets and array of (signal quality, angle (rad), distance (m))"
    sign = -1 if flipped else 1
    distances = [(quality, sign*2*pi*angle/360, distance/1000.0) for quality, angle, distance in scan_data]
    return distances

def look():
    """Fetch new distance data and update the buffer"""
    fill_measures(next(lidar_iter))
def spin(stop_check):
    """Keep fetching and updating buffer with distance data"""
    while not stop_check(): # You will notice there is no sleep to rest between loops.
        look()              # Thats cuz next(lidar_iter) is blocking for a while.
                            # Also cuz we are literally trying to consume as fast as possible 
                            # to prevent data build up in buffer
stop_spinning=False
lidar_thread = Thread(target=spin, args=(lambda: stop_spinning, ), name="Lidar_consumption_thread")
lidar_thread.start()
#====================================

#======================================
# Maintaining Measurements with History
#--------------------------------------
history_size = 10 # Hold 2 measurements in history
measurement_history = [[]]*history_size 
def update_measurement():
    global measurement_history
    measurement_history = [get_measures(),*measurement_history[:-1]]

#-------------------------------------------
# Joining Neighboring points into obstacles
# Maintain Obstacles in a thread (maybe I've created too many threads)

thresh = 1 # In meters
obstacles = None    # List of obstacles
                    # Each obstacle is a list of points
                    # None if not initialized
def point_distance(p1, p2):
    d1 = sqrt(abs(p1[1]**2 + p2[1]**2 - 2*p1[1]*p2[1]*cos(p1[0]-p2[0])))
    return d1
def join_points(points):
    piter = iter(points)
    try:
        groups = [[next(piter)]]
    except StopIteration:
        return None
    for p in piter:
        if point_distance(groups[-1][-1], p) > thresh: # last group's last point
            groups.append([])
        groups[-1].append(p)
    if point_distance(groups[0][0], groups[-1][-1]) < thresh and len(groups) > 1:
        groups[0] = groups[0] + groups[-1]
        groups.pop()
    return groups

#----------------------------------------------------------
# Start up the thread to maintain point cloud and obstacles

def keep_updating_obstacles(stop_updating):
    global obstacles
    while not stop_updating():
        time.sleep(0.05)
        update_measurement()
        measures = sum(measurement_history, [])
        measures = sorted(measures, key=lambda i: i[1])
        obstacles = join_points(((a,d) for q,a,d in measures)) 
stop_updating_obstacles = False
obstacle_thread = Thread(target=keep_updating_obstacles, args=(lambda: stop_updating_obstacles, ), name="Obstacle_maintaining_thread")
obstacle_thread.start()







try:
    fig = plt.figure()
    ax = plt.subplot(111, projection='polar')
    obstacle_points = ax.scatter([],[], s=1)
    obstacle_groups = LineCollection([], color=(0,0,0,0.5), linewidths=5)
    ax.add_collection(obstacle_groups)

    rmax=5
    rmax_=0.01
    ax.set_rmax(rmax)
    def update_plot(_):
        modded = []
        # Visualizing Point Cloud
        mes_his = list(reversed(measurement_history))                           # If someone
        measures = sum(mes_his, [])                                             # could just
        measurement_freshness = [ [i]*len(m) for i, m in enumerate(mes_his)]    # reduced the lines
        measurement_freshness = sum(measurement_freshness, [])                  # of code here
        alphas = [i/history_size for i in measurement_freshness]                # I'd love it      
        obstacle_points.set_offsets([(m[1], m[2]) for m in measures])           # 
        obstacle_points.set_color(list(zip(*[[0]*len(alphas)]*3, alphas)))      # 
        modded.append(obstacle_points)                                          #
        #---------------------
        # Visualizing Obstacles
        obstacle_groups.set_segments(obstacles)
        modded.append(obstacle_groups)
        #---------------------
        return modded

    anime = anim.FuncAnimation(fig, update_plot, 1, interval=50, blit=True)

    plt.show()
except Exception as e:
    traceback.print_exception(e)



stop_updating_obstacles=True
obstacle_thread.join()
stop_spinning=True
lidar_thread.join()
lidar.stop()
lidar.stop_motor()
lidar.disconnect()