import math
import json
import requests
import os, sys
sys.path.insert(0, os.path.abspath(".."))
from Autonomous_Systems.RoverNavigation import RoverNavigation
from Autonomous_Systems import AutoHelp
from modules.BN08x import BN08x as IMU
import time
import threading

class Autonomy:
    def __init__(self, rover_comms, url, max_speed, max_steering, GPS, GPS_coordinate_map, object_detector):
        self.rover_comms = rover_comms
        self.url = url
        self.IMU = IMU()
        self.GPS = GPS
        self.object_detector = object_detector

        self.RoverNavigation = RoverNavigation(max_speed, max_steering, self.GPS, self.IMU, GPS_coordinate_map)
        self.AutoHelp = AutoHelp.AutoHelp()

        self.current_GPS = None
        self.GPS_lock = threading.Lock()
        self.current_object_data = None
        self.object_data_lock = threading.Lock()

    def update_gps(self):
        while True:
            new_GPS = self.GPS.get_position()
            with self.GPS_lock:
                if new_GPS is not None:
                    self.current_GPS = new_GPS

    def update_object_data(self):
        for camera_boxes, lidar_data in self.object_detector.calculate_objects():
            with self.object_data_lock:
                self.current_object_data = (camera_boxes, lidar_data)


    def get_rover_status(self, bearing, distance):
        try:
            json_command = {"Bearing":bearing,"Distance":distance,"GPS":[self.current_GPS[0],self.current_GPS[1]],"Target":[self.RoverNavigation.GPS_target[0],self.RoverNavigation.GPS_target[1]]}
            json_command = json.dumps(json_command)
            json_command = json_command.replace(" ", "")
            requests.post(f"{self.url}/autonomy", json=json_command)
        except:
            print("Could not send rover status to mission control")

    def start_mission(self):
        gps_thread = threading.Thread(target=self.update_gps)
        gps_thread.start()

        object_detector_thread = threading.Thread(target=self.update_object_data)
        object_detector_thread.start()

        homing_end = {"HB":0,"IO":1,"WO":0,"DM":"D","CMD":[0,0]}
        while True:
            response = self.rover_comms.read_data()
            if homing_end in response:
                while True:
                    with self.GPS_lock:
                        current_GPS = self.current_GPS
                    if current_GPS and current_GPS != "Need More Satellite Locks":
                        # command = self.RoverNavigation.get_steering(current_GPS, self.RoverNavigation.GPS_target)
                        command = self.RoverNavigation.follow_path(self.RoverNavigation.GPS_target)
                        bearing = round(self.AutoHelp.get_bearing(current_GPS, self.RoverNavigation.GPS_target), 3)
                        distance = round(self.AutoHelp.get_distance(current_GPS, self.RoverNavigation.GPS_target)[0]*1000, 3)
                        quat_i, quat_j, quat_k, quat_real = self.IMU.get_rotation()
                        self.heading = self.IMU.get_heading(quat_real, quat_i, quat_j, quat_k)
                        print("Current GPS:", current_GPS)
                        print("Target GPS:", self.RoverNavigation.GPS_target)
                        print("Heading:", self.heading)
                        print("Bearing:", bearing)
                        print("Distance from Target GPS:", distance, "Meters")
                        print("Sending Command:", command)
                        response = self.rover_comms.read_data()
                        # self.get_rover_status(bearing, distance)
                        if response != "No data received" and command != None:
                            self.rover_comms.write_data(command)
                    else:
                        print("GPS Error. Current GPS:", current_GPS)
                        time.sleep(1)