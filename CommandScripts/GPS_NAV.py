import requests
import json
import os, sys
sys.path.insert(0, os.path.abspath(".."))
from modules.LSM303 import Compass
from CommandScripts import AutoHelp

class GPS_Nav:
    def __init__(self, url, max_speed, max_steering, GPS, GPS_coordinate_map):
        self.url = url
        self.max_speed = max_speed
        self.max_steering = max_steering
        self.commands = [0,0,0,'D',0,0]
        self.current_GPS = [0,0]
        self.compass = Compass.Compass()
        self.AutoHelp = AutoHelp.AutoHelp()
        self.GPS = GPS
        self.GPS_coordinate_map = GPS_coordinate_map
        self.GPS_target = self.GPS_coordinate_map[0]
        
    def forward_rover(self, commands):
        self.commands = [0, 0, 0, 'D', self.max_speed, 0]
        return self.jsonify_commands(commands)

    def steer_left(self, commands, steer_error):
        steer_output = self.steer_controller(steer_error)  # Set output to negative for left steering
        speed_output = -steer_output

        self.commands[4] = round(max(0, min(self.max_speed/2 + speed_output, self.max_speed)))
        self.commands[5] = round(max(-self.max_steering, min(steer_output, self.max_steering)))

        return self.jsonify_commands(commands)

    def steer_right(self, commands, steer_error):
        steer_output = self.steer_controller(steer_error)  # Set output to positive for right steering
        speed_output = -steer_output

        self.commands[4] = round(max(0, min(self.max_speed/2 + speed_output, self.max_speed)))
        self.commands[5] = abs(round(max(-self.max_steering, min(steer_output, self.max_steering))))

        return self.jsonify_commands(commands)

    def drive(self, commands, drive_error):
        speed_output = self.speed_controller(drive_error)

        self.commands[4] = round(max(0, min(self.max_speed + speed_output, self.max_speed)))
        self.commands[5] = 0

        return self.jsonify_commands(commands)

    def stop_rover(self, commands):
        self.commands = [0, 0, 0, 'D', 0, 0]
        return self.jsonify_commands(commands)

    def goto_next_coordinate(self):
        self.GPS_coordinate_map.pop(0)
        self.GPS_target = self.GPS_coordinate_map[0]


    def get_steering(self, current_GPS, GPS_target):
        rover_heading = self.compass.get_heading()
        bearing = self.AutoHelp.get_bearing(current_GPS, GPS_target)
        distance = round(self.get_distance(current_GPS, GPS_target)[0]*1000)

        direction = (bearing - rover_heading + 360) % 360
        direction = direction+100
        print("Direction:", direction)

        if abs(direction) > 10:

            if direction < 120:
                print("Turning right")
                return self.steer_right(self.commands, direction)
            else:
                print("Turning left")
                return self.steer_left(self.commands, direction)
        rover_heading = bearing
        
        if distance > 0.1:
            print("Moving forward")
            return self.drive(self.commands, direction)
        else:
            print("Arrived at target!")
            self.goto_next_coordinate()
            return self.stop_rover(self.commands)


    def get_rover_status(self):
        bearing = self.get_bearing(self.current_GPS, self.GPS_target)
        distance = round(self.get_distance(self.current_GPS, self.GPS_target)[0]*1000)
        json_command = {"Bearing":bearing,"Distance":distance,"GPS":[self.current_GPS[0],self.current_GPS[1]],"Target":[self.GPS_target[0],self.GPS_target[1]]}
        json_command = json.dumps(json_command)
        json_command = json_command.replace(" ", "")
        requests.post(f"{self.url}/autonomy/status", json=json_command)