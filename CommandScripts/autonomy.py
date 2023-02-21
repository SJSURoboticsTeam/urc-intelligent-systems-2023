import math
import json
import requests
from CommandScripts import Trajectory
import os, sys
sys.path.insert(0, os.path.abspath(".."))
from CommandScripts.GPS_NAV import GPS_Nav
from CommandScripts import AutoHelp
from modules.LSM303 import Compass
import time

class Autonomy:
    def __init__(self, serial, url, max_speed, max_steering, GPS, GPS_coordinate_map):
        self.serial = serial
        self.url = url
        self.compass = Compass()
        self.GPS = GPS

        self.GPS_Nav = GPS_Nav(self.url, max_speed, max_steering, self.GPS, self.compass, GPS_coordinate_map)
        self.AutoHelp = AutoHelp.AutoHelp()


    def get_rover_status(self, bearing, distance):
        json_command = {"Bearing":bearing,"Distance":distance,"GPS":[self.current_GPS[0],self.current_GPS[1]],"Target":[self.GPS_Nav.GPS_target[0],self.GPS_Nav.GPS_target[1]]}
        json_command = json.dumps(json_command)
        json_command = json_command.replace(" ", "")
        requests.post(f"{self.url}/autonomy", json=json_command)


    def start_mission(self):
        # Uncomment this below for testing on the Rover
        homing_end = "Starting control loop..."
        while True:
            response = self.serial.read_serial()
            if homing_end in response:
                while True:
                    self.current_GPS = self.GPS.get_position()
                    # self.current_GPS = [-121.88263100000002, 37.33752616666666]
                    if self.current_GPS != "Need More Satellite Locks" and self.current_GPS != None:

                        command = self.GPS_Nav.get_steering(self.current_GPS, self.GPS_Nav.GPS_target)
                        bearing = round(self.AutoHelp.get_bearing(self.current_GPS, self.GPS_Nav.GPS_target), 3)
                        distance = round(self.AutoHelp.get_distance(self.current_GPS, self.GPS_Nav.GPS_target)[0]*1000, 3)
                        print("Current GPS:", self.current_GPS)
                        print("Target GPS:", self.GPS_Nav.GPS_target)
                        print("Heading:", self.compass.get_heading())
                        print("Bearing:", bearing)
                        print("Distance from Target GPS:", distance, "Meters")
                        print("Sending Command:", command)
                        response = self.serial.read_serial()
                        self.get_rover_status(bearing, distance)
                        if response != "No data received" and command != None:
                            self.serial.write_serial(command)
                        else:
                            continue
                    else:
                        print("GPS Error. Current GPS:", self.current_GPS)