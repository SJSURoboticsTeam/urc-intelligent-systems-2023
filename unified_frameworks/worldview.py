"""
This code is responsible for reading in all sensory inputs 
and unifying them into a single worldview for everything else
to access
"""
import time
from threading import Thread
from sensor_array import lidar

config = {
    "update_frequency": 20, #Hz
    "service_event_verbose": True,
}
_obstacles = None # List of points clustered into obstacles (radians,meters)
def run_worldview(service_is_active):
    if config["service_event_verbose"]:
        print("Starting Worldview Service")
    lidar.start_lidar_service()
    global _obstacles
    while service_is_active():
        _obstacles = lidar.get_obstacles()
        time.sleep(1/config['update_frequency'])
    lidar.stop_lidar_service()
    if config["service_event_verbose"]:
        print("Starting Worldview Service")

def get_obstacles():
    return _obstacles

_thread = None
_running = False
def start_worldview_service():
    if config["service_event_verbose"]:
        print("Called start_worldview_service")
    global _thread, _running
    if _running:
        raise Exception("Tried to Start Worldview Service but it was already running.")
    _thread = Thread(target=run_worldview, args=(lambda: _running,), name="Worldview Service Thread")
    _running = True
    _thread.start()

def stop_worldview_service():
    if config["service_event_verbose"]:
        print("Called stop_worldview_service")
    global _running, _thread
    _running = False
    _thread.join()

def worldview_service_is_running():
    return _running


if __name__=="__main__":
    start_worldview_service()
    time.sleep(10)
    stop_worldview_service()
