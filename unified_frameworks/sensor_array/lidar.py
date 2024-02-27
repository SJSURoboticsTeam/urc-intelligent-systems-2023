from rplidar import RPLidar, RPLidarException
import traceback
import sys
from threading import Thread
from math import pi, cos, sin, sqrt
import time
import os
import json

config = {
    "service_file": "lidar.txt",
    "update_frequency": 20, # Hz
    "history_size": 7,
    "service_event_verbose":True,
    "verbose_lidar_exceptions":False
}
_point_clouds = None # This will be the raw point cloud
_obstacles = None # This will be the points clustered into obstacles
def run_lidar(service_is_active):
    if config["service_event_verbose"]:
        print("Starting Lidar Service")
    #==================================
    # Initial connect and set up Lidar
    #----------------------------------
    PORT_NAME = 'COM10'
    lidar = None
    lidar_iter = None
    ts = time.time()
    while service_is_active():
        if config['service_event_verbose'] and time.time()-ts > 1:
            ts = time.time()
        try: # Try and try again until Lambs become lions and 
            lidar = RPLidar(PORT_NAME)
            # lidar_iter = iter(lidar.iter_scans(max_buf_meas=10000)) # Large buffer is able to hold on to data for longer before yielding it. This means the data received can older (Therefore laggier)
            lidar_iter = iter(lidar.iter_scans()) # Stick to the default buffer size
            next(lidar_iter) # and until the Lidar is able to yield stuff without errors
        except RPLidarException as e:
            if config['verbose_lidar_exceptions']:
                print("======================RPLidarException================================")
                print(e)
            lidar.disconnect()
            continue
        except:
            print("Lidar Service Failed before lidar could start")
            print(traceback.format_exc())
            sys.exit(1)
        print()
        break
    #====================================

    #====================================
    # Running the Lidar
    #------------------------------------
    scanned_data = list() # buffer to hold distance data # use same ters scan/measures/
    def set_scan(distances):
        "Sets and array of (signal quality, angle (rad), distance (m))"
        nonlocal scanned_data
        scanned_data = distances
    def get_scan(flipped=True):
        "Gets and array of (signal quality, angle (rad), distance (m))"
        sign = -1 if flipped else 1
        distances = [(quality, sign*2*pi*angle_deg/360, distance_mm/1000.0) for quality, angle_deg, distance_mm in scanned_data]
        return distances

    def look():
        """Fetch new distance data and update the buffer"""
        set_scan(next(lidar_iter))
    def spin(stop_check):
        """Keep fetching and updating buffer with distance data"""
        while not stop_check(): # You will notice there is no sleep to rest between loops.
            look()              # Thats cuz next(lidar_iter) is blocking for a while.
                                # Also cuz we are literally trying to consume as fast as possible 
                                # to prevent data build up in buffer
    _stop_spinning=False
    lidar_thread = Thread(target=spin, args=(lambda: _stop_spinning, ), name="Lidar_consumption_thread")
    lidar_thread.start()
    if config['service_event_verbose']:
        print("started Lidar Scanning Thread")
    #====================================

    #======================================
    # Maintaining Measurements with History
    #--------------------------------------
    history_size = config['history_size']
    measurement_history = [[]]*history_size # A list of history_size empty lists
    def update_measurement():
        nonlocal measurement_history
        measurement_history = [get_scan(),*measurement_history[:-1]]

    #-------------------------------------------
    # Joining Neighboring points into obstacles
    # Maintain Obstacles in a thread (maybe I've created too many threads)

    thresh = 1 # In meters
    obstacles = None    # List of obstacles
                        # Each obstacle is a list of points
                        # None if not initialized
    def calc_polar_distance(p1, p2):
        d1 = sqrt(abs(p1[1]**2 + p2[1]**2 - 2*p1[1]*p2[1]*cos(p1[0]-p2[0])))
        return d1
    def join_points(points):
        piter = iter(points)
        try:
            groups = [[next(piter)]]
        except StopIteration:
            return None
        for p in piter:
            if calc_polar_distance(groups[-1][-1], p) > thresh: # last group's last point
                groups.append([])
            groups[-1].append(p)
        if calc_polar_distance(groups[0][0], groups[-1][-1]) < thresh and len(groups) > 1:
            groups[0] = groups[-1] + groups[0]
            groups.pop()
        return groups

    #----------------------------------------------------------
    # Start up the thread to maintain point cloud and obstacles

    def keep_updating_obstacles(stop_updating):
        nonlocal obstacles
        while not stop_updating():
            time.sleep(1/config["update_frequency"])
            update_measurement()
            measures = sum(measurement_history, []) # use [].extend(measurement for clarity)
            angle_sorted_measures = sorted(measures, key=lambda i: i[1])
            obstacles = join_points(((a,d) for q,a,d in angle_sorted_measures))  #quality, angle, distance
    stop_updating_obstacles = False
    obstacle_thread = Thread(target=keep_updating_obstacles, args=(lambda: stop_updating_obstacles, ), name="Obstacle_maintaining_thread")
    obstacle_thread.start()


    # This could probably be put in the same thread as obstacle thread
    # i = 0
    global _point_clouds, _obstacles
    while service_is_active():
        _point_clouds = [
            [(a,d) for q,a,d in measures]
            for measures in reversed(measurement_history)
        ]
        _obstacles = obstacles
        # with(open(config["service_file"], 'w')) as f:
        #     f.write(json.dumps(obstacles))
        # print(i)
        # i+=1
        time.sleep(1/config['update_frequency'])
    # os.remove(config["service_file"])


    stop_updating_obstacles=True
    if config['service_event_verbose']:
        print("waiting for obstacle thread to join")
    obstacle_thread.join()
    _stop_spinning=True
    if config['service_event_verbose']:
        print("waiting for lidar thread to join")
    lidar_thread.join()
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    if config["service_event_verbose"]:
        print("Stopping Lidar Service")

def get_point_clouds():
    """
    Get None or the past history_size number of point clouds measured by the lidar 
    where each point is represented by (radians, meter)
    The history is returned most recent first
    """
    return _point_clouds
def get_obstacles():
    """
    Get None or a list of obstacles recognized by the lidar. 
    Obstacles will be a list of (radian, meter) points.
    """
    return _obstacles



_thread = None
_running = False
def start_lidar_service():
    if config['service_event_verbose']:
        print("Called start_lidar_service")
    global _thread, _running
    if _running:
        raise Exception("Tried to Start Lidar Service but it was already running.")
    _thread = Thread(target=run_lidar, args=(lambda: _running,), name="Lidar Service Thread")
    _running = True
    _thread.start()

def stop_lidar_service():
    if config['service_event_verbose']:
        print("Called stop_lidar_service")
    global _running, _thread
    _running = False
    _thread.join()

def lidar_service_is_running():
    return _running

if __name__=='__main__':
    start_lidar_service()
    # time.sleep(30)
    for _ in range(7):
        # print(get_point_clouds())
        time.sleep(1)
    stop_lidar_service()