import math
import json
import requests
import os, sys
sys.path.insert(0, os.path.abspath(".."))
import time

class Autonomy:
    def __init__(self, serial, url, GPS, GPS_coordinate_map):
        self.serial = serial
        self.url = url
        self.commands = [0,0,0,'D',0,0]
        self.current_GPS = [0,0]
        self.GPS = GPS
        self.gain = 1

    def start_mission(self):
        # homing_end = "Starting control loop..."
        # while True:
        #     response = self.serial.read_serial()
        #     if homing_end in response:
                while True:
                    self.current_GPS = self.GPS.get_position()
                    print("Current GPS:", self.current_GPS)
                    print("Target GPS:", self.GPS_target)
                    if self.current_GPS != "Need More Satellite Locks" and self.current_GPS != None:
                        command = self.get_steering(self.current_GPS, self.GPS_target)
                        distance = round(self.get_distance(self.current_GPS, self.GPS_target)[0]*1000)
                        print("Distance from Target GPS:", distance, "Meters")
                        print("Sending Command:", command)
                        response = self.serial.read_serial()
                        self.get_rover_status()
                        if response != "No data received" and command != None:
                            self.serial.write_serial(command)
                            time.sleep(1)
                        else:
                            continue
                    else:
                        print("GPS Error. Current GPS:", self.current_GPS)
