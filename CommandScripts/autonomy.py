import math
import json
import requests
import os, sys
sys.path.insert(0, os.path.abspath(".."))
from modules.LSM303 import Compass
import serial.tools.list_ports as port_list

class Autonomy:
    def __init__(self, serial, url, max_speed, max_steering, GPS, GPS_coordinate_map):
        self.serial = serial
        self.url = url
        self.max_speed = max_speed
        self.max_steering = max_steering
        self.commands = [0,0,0,'D',0,0]
        self.current_GPS = [0,0]
        self.compass = Compass()
        self.GPS = GPS
        self.gain = 1
        self.GPS_coordinate_map = GPS_coordinate_map
        self.GPS_target = self.GPS_coordinate_map[0]
        self.GPS = GPS



                    

    def get_distance(self, current_GPS, target_GPS):

        R_KM = 6373.0
        R_MI = 3958.8
        current_lat = math.radians(current_GPS[1])
        current_lon = math.radians(current_GPS[0])
        target_lat = math.radians(target_GPS[1])
        target_lon = math.radians(target_GPS[0])

        dis_lon = target_lon - current_lon
        dis_lat = target_lat - current_lat

        form1 = math.sin(dis_lat / 2)**2 + math.cos(current_lat) * math.cos(target_lat) * math.sin(dis_lon / 2)**2
        form2 = 2 * math.atan2(math.sqrt(form1), math.sqrt(1 - form1))

        distanceKM = R_KM * form2
        distanceMi = R_MI * form2
        return [distanceKM, distanceMi]

    def get_spin_angle(self, current_GPS, target_GPS):
            x = target_GPS[1] - current_GPS[1]
            y = target_GPS[0] - current_GPS[0]
            quad = 0
            angle = 0

            compass = Compass()
            heading = compass.get_heading()

            if x < 0:
                if y < 0:
                    quad = 3
                else:
                    quad = 2
            else:
                if y < 0:
                    quad = 4
                else:
                    quad = 1
                    
            angle += 360-heading

            if quad == 1:
                angle += 90-angle
                angle -= math.degrees(math.atan(abs(y)/abs(x)))
            elif quad == 2:
                angle += 360-angle
                angle -= math.degrees(math.atan(abs(y)/abs(x)))
            elif quad == 3:
                angle += 270-angle
                angle -= 90 - math.degrees(math.atan(abs(y)/abs(x)))
            elif quad == 4:
                angle += 180-angle
                angle -= 90 - math.degrees(math.atan(abs(y)/abs(x)))

            if angle < (360-angle):
                return angle
            else:
                return -abs(360-angle)

    def jsonify_commands(self, commands):
        json_command = {"HB":commands[0],"IO":commands[1],"WO":commands[2],"DM":f"{commands[3]}","CMD":[commands[4],commands[5]]}
        json_command = json.dumps(json_command)
        json_command = json_command.replace(" ", "")
        return json_command


    def get_bearing(self, current_GPS, target_GPS):
        current_latitude = math.radians(current_GPS[1])
        current_longitude = math.radians(current_GPS[0])
        target_latitude = math.radians(target_GPS[1])
        target_longitude = math.radians(target_GPS[0])


        deltalog= target_longitude-current_longitude;

        x=math.cos(target_latitude)*math.sin(deltalog);
        y=(math.cos(current_latitude)*math.sin(target_latitude))-(math.sin(current_latitude)*math.cos(target_latitude)*math.cos(deltalog));

        bearing=(math.atan2(x,y))*(180/3.14);
        return bearing


    def forward_rover(self, commands):
        self.commands[4] = self.max_speed
        self.jsonify_commands(commands)

    def steer_left(self, commands):
        self.commands[5] = -self.max_steering
        self.jsonify_commands(commands)

    def steer_right(self, commands):
        self.commands[5] = self.max_steering
        self.jsonify_commands(commands)

    def stop_rover(self, commands):
        self.commands = [0,0,0,'D',0,0]
        self.jsonify_commands(commands)

    def goto_next_coordinate(self):
        self.GPS_coordinate_map.pop(0)
        self.GPS_target = self.GPS_coordinate_map[0]


    def get_steering(self, current_GPS, target_GPS):
        
        final_angle = self.compass.get_heading()/self.get_bearing(current_GPS, target_GPS)

        if(final_angle >= 0 and final_angle <= 1):
            print("Rover moving forward!")
            self.forward_rover(self.commands)
            
        elif(final_angle > 1 and final_angle <= 8):
            print("Rover turning left!")
            self.steer_left(self.commands)
            

        elif(final_angle <= 13 and final_angle >= 8):
            print("Rover turning right!")
            self.steer_right(self.commands)
            

        elif(target_GPS[0]==current_GPS[0] and current_GPS[1]==target_GPS[1]):
            print("Rover has reached destination!")
            self.stop_rover(self.commands)
            self.goto_next_coordinate()

    def forward_gain_rover(self, commands,error):
        self.commands[4] = error*self.gain
        self.jsonify_commands(commands)
    
    def set_gain(self,ingain):
        self.gain = ingain

    def steer_gain_left(self, commands, error):
        self.commands[5] = -error*self.gain
        self.jsonify_commands(commands)

    def steer_gain_right(self, commands, error):
        self.commands[5] = error*self.gain
        self.jsonify_commands(commands)

    def get_ctl_steer(self, current_GPS, target_GPS):
        bearing = self.get_bearing(current_GPS, target_GPS)
        dist = self.get_distance(current_GPS, target_GPS)
        # maximum deviation allowed by bearing
        threshold = 1
        if bearing >= threshold:
            if(bearing > 180):
                self.steer_gain_left(self.commands,bearing)
            else:
                self.steer_gain_right(self.commands,bearing)
        else:
            if(target_GPS[0]==current_GPS[0] and current_GPS[1]==target_GPS[1]):
                print("Rover has reached destination!")
                self.stop_rover(self.commands)
                self.goto_next_coordinate()
            else:
                self.forward_gain_rover(dist)


    def get_rover_status(self):
        bearing = self.get_bearing(self.current_GPS, self.GPS_target)
        distance = self.get_distance(self.current_GPS, self.GPS_target)
        json_command = {"Bearing":bearing,"Distance":distance[0],"GPS":[self.current_GPS[0],self.current_GPS[1]],"Target":[self.GPS_target[0],self.GPS_target[1]]}
        json_command = json.dumps(json_command)
        json_command = json_command.replace(" ", "")
        requests.post(self.url, data=None, json=json_command)


    def start_mission(self):
        while True:
            self.current_GPS = self.GPS.get_position(f"{self.url}/gps")
            if self.current_GPS != "Need More Satellite Locks":
                command = self.get_steering(self.current_GPS, self.GPS_target)
                # command = self.get_ctl_steering(self.current_GPS, self.GPS_target)
                response = self.serial.read_serial()
                self.get_rover_status()
                if response != "No data received":
                    self.serial.read_write_serial(command.text)
                else:
                    continue
            else:
                print(self.current_GPS)
