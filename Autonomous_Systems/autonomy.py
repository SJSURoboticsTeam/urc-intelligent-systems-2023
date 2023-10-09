import math
import json
import requests
from Autonomous_Systems import Trajectory
import os, sys
sys.path.append('../')
from Autonomous_Systems import RoverNavigation
from Autonomous_Systems import AutoHelp
from modules.LSM303 import Compass
from modules.WiFi import WiFi
import time
import threading

class Autonomy:
    def __init__(self, rover_comms, url, max_speed, max_steering, GPS, GPS_coordinate_map):
        self.rover_comms = rover_comms
        self.url = url
        self.compass = Compass()
        self.GPS = GPS

        self.RoverNavigation = RoverNavigation(max_speed, max_steering, self.GPS, self.compass, GPS_coordinate_map)
        self.AutoHelp = AutoHelp.AutoHelp()

        self.current_GPS = None
        self.GPS_lock = threading.Lock()

    def update_gps(self):
        while True:
            new_GPS = self.GPS.get_position()
            with self.GPS_lock:
                if new_GPS is not None:
                    self.current_GPS = new_GPS


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

        homing_end = "Starting control loop..."
        while True:
            response = self.rover_comms
            if homing_end in response:
                while True:
                    with self.GPS_lock:
                        current_GPS = self.current_GPS
                    if current_GPS and current_GPS != "Need More Satellite Locks":
                        command = self.RoverNavigation.get_steering(current_GPS, self.RoverNavigation.GPS_target)
                        bearing = round(self.AutoHelp.get_bearing(current_GPS, self.RoverNavigation.GPS_target), 3)
                        distance = round(self.AutoHelp.get_distance(current_GPS, self.RoverNavigation.GPS_target)[0]*1000, 3)
                        print("Current GPS:", current_GPS)
                        print("Target GPS:", self.RoverNavigation.GPS_target)
                        print("Heading:", self.compass.get_heading())
                        print("Bearing:", bearing)
                        print("Distance from Target GPS:", distance, "Meters")
                        print("Sending Command:", command)
                        response = WiFi.read_data()
                        self.get_rover_status(bearing, distance)
                        if response != "No data received" and command != None:
                            WiFi.write_data(command)
                    else:
                        print("GPS Error. Current GPS:", current_GPS)
                        time.sleep(1)