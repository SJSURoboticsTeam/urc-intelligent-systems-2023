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
scan_data = list() # buffer to hold distance data
def fill_measures(distances):
    global scan_data
    scan_data = distances
def get_measures(flipped=True):
    sign = -1 if flipped else 1
    distances = [(quality, sign*2*pi*angle/360, distance/1000.0) for quality, angle, distance in scan_data]
    return distances

def look():
    """Fetch new distance data and update the buffer"""
    fill_measures(next(lidar_iter))
def spin(stop_check):
    """Keep fetching and updating buffer with distance data"""
    while not stop_check():
        look()
stop_spinning=False
lidar_thread = Thread(target=spin, args=(lambda: stop_spinning, ))
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

#==========================================
# Joining Neighboring points into obstacles
#------------------------------------------
thresh = 0.01
def point_distance(p1, p2):
    d = sqrt(p1[0]**2 + p2[0]**2 + 2*p1[0]*p2[0]*cos(p1[1]-p2[1]))
    print(d)
    return d
def join_points(points):
    return []
    groups = [[points[0]]]
    for p in points[1:]:
        if point_distance(groups[-1][-1], p) < thresh: # last group's last point
            groups.append([])
        groups[-1].append(p)
    if point_distance(groups[0][0], groups[-1][-1]) > thresh and len(groups) > 1:
        groups[0] = groups[0] + groups[-1]
        groups.pop()
    return groups




try:
    fig = plt.figure()
    ax = plt.subplot(111, projection='polar')
    obstacle_points = ax.scatter([],[], s=5, c=[], cmap=plt.cm.Greys)
    obstacle_groups = LineCollection([])
    ax.add_collection(obstacle_groups)




    rmax=5
    rmax_=0.01
    ax.set_rmax(rmax)
    def update_plot(_):
        modded = []
        update_measurement()
        # Visualizing Point Cloud
        mes_his = list(reversed(measurement_history))
        measures = sum(mes_his, [])
        measurement_freshness = [ [i]*len(m) for i, m in enumerate(mes_his)]
        measurement_freshness = sum(measurement_freshness, [])
        obstacle_points.set_offsets([(m[1], m[2]) for m in measures])
        obstacle_points.set_array(measurement_freshness)
        modded.append(obstacle_points)
        #---------------------
        # Visualizing Obstacles
        # print(measurement_history)
        segments = join_points([(ang,dist) for _,ang,dist in measurement_history[0]])
        obstacle_groups.set_segments(segments)
        # print(segments)
        # print()
        modded.append(obstacle_groups)
        #---------------------
        return modded

    anime = anim.FuncAnimation(fig, update_plot, 1, interval=50, blit=True)

    plt.show()
except Exception as e:
    traceback.print_exception(e)






# try:
#     fig, axs = plt.subplots(ncols=2, subplot_kw={'projection': 'polar'})
#     axl: Axes = axs[0]
#     # axl.axis('off')
#     axl.set_title('Upside_down_lidar')
#     axl.set_rmax(1)
#     # axl.set_rmin(0)
#     scatL = axl.scatter([], [], marker='o', c='orange')
#     obs_collectionL = LineCollection([])
#     axl.add_collection(obs_collectionL)
#     axl.set_visible(False)

#     axr: Axes = axs[1]
#     # axr.axis('off')
#     axr.set_title('Upside_up_lidar')
#     axr.set_rmax(1) # All distances will be scaled to a 0-1 range
#     # axr.set_rmin(0)
#     scatR = axr.scatter([], [], marker='o', c='orange')
#     obs_collectionR = LineCollection([])
#     axr.add_collection(obs_collectionR)
#     def join_near_points(points, flipped = False, thresh=0.2):
#         """Given points in polar coordinates,
#         Returns a 2D List of obstacles, 
#         where each obstacle is a collection of points near each other"""
#         obstacles = [[(0,points[0])]]
#         for i, p in enumerate(points[1:]):
#             if abs(p-obstacles[-1][-1][1]) > thresh: # If point p is thresh far away from the last point in the last obstacle, then it is part of a new obstacle
#                 obstacles.append([])
#             angle  = 2*pi/360*(i+1) if not flipped else 2*pi/360*(360-i+1)
#             obstacles[-1].append((angle,p))
#         # check if first and last obstacles are actually the same obstacles
#         if (obstacles[-1][-1][1]-obstacles[0][0][1] <= thresh):
#             obstacles[0] = obstacles[-1] + obstacles[0]
#             obstacles.pop()
#         print()
#         print(obstacles)
#         print()
#         return obstacles
#     def update(_):
#         modded = []
#         # scan_data_rev = list(reversed(scan_data))
#         # RMAX=max(scan_data)*1.2
#         # axr.set_rmax(RMAX)
#         # scatR.set_offsets([(2*pi/360*(359-i), scan_data[i]) for i in range(0,360)])
#         # modded.append(scatR)
#         maxR = max(scan_data)*1.1
#         scaled_data = [i/maxR for i in scan_data]
#         obs_collectionR.set_segments(join_near_points(scaled_data, True))
#         modded.append(obs_collectionR)

#         # axl.set_rmax(RMAX)
#         # scatL.set_offsets([(2*pi/360*i, scan_data[i]) for i in range(0,360)])
#         # modded.append(scatL)

#         # obs_collectionL.set_segments(join_near_points(scaled_data))
#         # modded.append(obs_collectionL)

#         return modded

#     anime = anim.FuncAnimation(
#         fig,
#         update,
#         1,
#         interval=1,
#         blit=True
#     )
#     plt.show()
# except Exception as e:
#     print(e)





stop_spinning=True
lidar_thread.join()
lidar.stop()
lidar.stop_motor()
lidar.disconnect()