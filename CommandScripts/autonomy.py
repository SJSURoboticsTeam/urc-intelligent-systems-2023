import math
import json
import requests
import os, sys
sys.path.insert(0, os.path.abspath(".."))
from modules.LSM303 import Compass
import time

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
        self.steer_error_sum = 0
        self.steer_prev_error = 0
        self.speed_error_sum = 0
        self.speed_prev_error = 0
        self.steer_kp = 0.2
        self.steer_ki = 0.001
        self.steer_kd = 0.1
        self.speed_kp = 0.2
        self.speed_ki = 0.001
        self.speed_kd = 0.1


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
        self.commands = [0, 0, 0, 'D', self.max_speed, 0]
        return self.jsonify_commands(commands)

    def steer_left(self, commands):
        steer_error = -self.max_steering
        self.steer_error_sum += steer_error
        steer_derivative = steer_error - self.steer_prev_error
        steer_output = self.steer_kp * steer_error + self.steer_ki * self.steer_error_sum + self.steer_kd * steer_derivative
        self.steer_prev_error = steer_error

        speed_error = -steer_output
        self.speed_error_sum += speed_error
        speed_derivative = speed_error - self.speed_prev_error
        speed_output = self.speed_kp * speed_error + self.speed_ki * self.speed_error_sum + self.speed_kd * speed_derivative
        self.speed_prev_error = speed_error

        self.commands[4] = round(max(0, min(self.max_speed + speed_output, self.max_speed)))
        self.commands[5] = round(max(-self.max_steering, min(steer_output, self.max_steering)))
        return self.jsonify_commands(commands)

    def steer_right(self, commands):
        steer_error = self.max_steering
        self.steer_error_sum += steer_error
        steer_derivative = steer_error - self.steer_prev_error
        steer_output = self.steer_kp * steer_error + self.steer_ki * self.steer_error_sum + self.steer_kd * steer_derivative
        self.steer_prev_error = steer_error

        speed_error = -steer_output
        self.speed_error_sum += speed_error
        speed_derivative = speed_error - self.speed_prev_error
        speed_output = self.speed_kp * speed_error + self.speed_ki * self.speed_error_sum + self.speed_kd * speed_derivative
        self.speed_prev_error = speed_error

        self.commands[4] = round(max(0, min(self.max_speed + speed_output, self.max_speed)))
        self.commands[5] = round(max(-self.max_steering, min(steer_output, self.max_steering)))
        return self.jsonify_commands(commands)

    def stop_rover(self, commands):
        self.commands = [0, 0, 0, 'D', 0, 0]
        return self.jsonify_commands(commands)

    def goto_next_coordinate(self):
        self.GPS_coordinate_map.pop(0)
        self.GPS_target = self.GPS_coordinate_map[0]


    def get_steering(self, current_GPS, target_GPS):
        # heading = self.compass.get_heading()
        # bearing = self.get_bearing(current_GPS, target_GPS)

        # final_angle = math.degrees(math.atan2(math.sin(math.radians(bearing - heading)), math.cos(math.radians(bearing - heading))))
        final_angle = self.compass.get_heading()/self.get_bearing(current_GPS, target_GPS)


        if final_angle < 0:
            final_angle += 360

        print("Final Angle:", final_angle)

        if final_angle <= 1 or final_angle > 359:
            print("Rover moving forward!")
            return self.forward_rover(self.commands)

        elif final_angle > 1 and final_angle <= 180:
            print("Rover turning left!")
            return self.steer_left(self.commands)

        elif final_angle > 180 and final_angle < 359:
            print("Rover turning right!")
            return self.steer_right(self.commands)
        
        if current_GPS == target_GPS:
            print("Rover has reached destination!")
            self.goto_next_coordinate()
            return self.stop_rover(self.commands)


    def get_rover_status(self):
        bearing = self.get_bearing(self.current_GPS, self.GPS_target)
        distance = round(self.get_distance(self.current_GPS, self.GPS_target)[0]*1000)
        json_command = {"Bearing":bearing,"Distance":distance,"GPS":[self.current_GPS[0],self.current_GPS[1]],"Target":[self.GPS_target[0],self.GPS_target[1]]}
        json_command = json.dumps(json_command)
        json_command = json_command.replace(" ", "")
        requests.post(f"{self.url}/autonomy/status", json=json_command)


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
