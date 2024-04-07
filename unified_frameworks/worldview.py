"""
This code is responsible for reading in all sensory inputs 
and unifying them into a single worldview for everything else
to access
"""
import time
from threading import Thread
from math import pi
from unified_utils import polar_sum, _Abstract_Service
import sensor_array.lidar
import bridge.client_side
import importlib
importlib.reload(bridge.client_side)
importlib.reload(sensor_array.lidar)
lidar = sensor_array.lidar


config = {
    "update_frequency": 20, #Hz
    "service_event_verbose": True,
    "rover_body": [(-pi/2, 0.5), (pi, 0.25), (0, 0.25)],
    "verbose": True
}

class Worldview(_Abstract_Service):
    def __init__(self) -> None:
        """Set up all the sensors to feed their data into this worldview.
        
        """
        super().__init__()
        if config['verbose']: print("Initializing Worldview")
        self._lidar = lidar.Lidar()
    
    def start_service(self):
        """Start sensors and start unifying data from all sensors
        
        """
        self._lidar.start_service()
    def stop_service(self):
        """Stop unifying data and stop all sensors
        
        """
        self._lidar.stop_service()
    
    def get_obstacles(self):
        """Get a list of obstacles where each obstacle is a point cloud of the 
        obstacle
        
        """
        return self._lidar.get_obstacles()
    
    def get_points(self):
        """Get a point cloud of the environment. The point cloud is a list of 
        ``(radians, meters)``
        
        """
        return sum(self._lidar.get_point_clouds(), [])
    
    def get_rover_body():
        """Get the coordinates of the rovers body
        
        """
        return config["rover_body"]

if __name__=="__main__":
    w = Worldview()
    w.start_service()
    time.sleep(10)
    w.stop_service()
