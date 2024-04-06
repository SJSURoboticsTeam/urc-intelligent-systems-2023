import sys
import re
root = (next(re.finditer(".*unified_frameworks", __file__)).group())
sys.path.append(root) if root not in sys.path else None
from sensor_array.actual_lidar import ActualLidar
from sensor_array.bridge_lidar import BridgeLidar
import importlib
from sensor_array.fake_lidar import FakeLidar
from sensor_array.LidarClass import _Lidar
import traceback
from threading import Thread
from math import pi, cos, sin, sqrt, atan2, radians
import time
import json
import serial.tools.list_ports
import serial
from unified_utils import _Abstract_Service, polar_dis


config = {
    "lidar_preference": [ActualLidar, BridgeLidar, FakeLidar],
    "update_frequency": 20, # Hz
    "history_size": 1,
    "rover_radius": 0.7,
    "open_sector": [-pi/4, 5*pi/4],
    "point_buffer_meters": 1,
    "point_buffer_count": 0,
    "service_event_verbose":True,
    "verbose_lidar_exceptions":True,
    "lidar_port": "ws://192.168.1.130:8765", #getDevicePort(),
    "wireless_uri": "ws://192.168.1.130:8765",
    "verbose": True
}

class NoLidarException(Exception):
    def __init__(self, *args: object) -> None:
        super().__init__(*args)

class Lidar(_Abstract_Service):
    def __init__(self, preference=config['lidar_preference']) -> None:
        """Attempt to connect to the lidars listed in the preference and hold on
        to the lidar that connected. Throws ``NoLidarException`` if all lidars 
        fail to connect.
        
        """
        super().__init__()
        if config["verbose"]: print("Initializing Lidar")
        lidar = None
        for _lidar in preference:
            print(f"Trying to use {_lidar}")
            try:
                L:_Lidar = _lidar()
                if L.connect(verbose_attempts=True):
                    lidar = L
                    break
            except:
                pass

        if lidar is None:
            raise NoLidarException(f"Failed to connect on the following lidars {preference}")
        self._lidar: _Lidar = lidar
        self._lidar.disconnect()
    
    def start_service(self):
        """Connect to the Lidar and start scanning"""
        self._lidar.connect()
    def stop_service(self):
        """Stop scanning and disconnect from the lidar"""
        self._lidar.disconnect()

    def get_point_clouds(self):
        """
        Get a list of point Clouds of from the previous few Lidar scans. Each 
        point cloud is a list of ``(radians, meters)``
        
        """
        return [[(radians(deg),mm/1000) for q,deg,mm in self._lidar.get_measures()]]
    
    def get_obstacles(self, thresh=1):
        """
        Get a list of obstacles where each obstacle is a point cloud of the 
        obstacle

        :param thresh: Points closer than the threshold distance are grouped in 
            the same obstacle
        
        """
        measures = sum(self.get_point_clouds(), []) # use [].extend(measurement for clarity)
        angle_sorted_measures = sorted(measures, key=lambda i: i[0])

        piter = iter(angle_sorted_measures)
        try:
            groups = [[next(piter)]]
        except StopIteration:
            return None
        for p in piter:
            if polar_dis(groups[-1][-1], p) > thresh: # last group's last point
                groups.append([])
            groups[-1].append(p)
        if polar_dis(groups[0][0], groups[-1][-1]) < thresh and len(groups) > 1:
            groups[0] = groups[-1] + groups[0]
            groups.pop()
        
        return groups

    pass

if __name__=='__main__':
    l = Lidar()
    l.start_service()
    time.sleep(10)
    for _ in range(7):
        print()
        print(len(l.get_point_clouds()[0]), len(l.get_obstacles()))
        time.sleep(1)
    l.stop_service()
