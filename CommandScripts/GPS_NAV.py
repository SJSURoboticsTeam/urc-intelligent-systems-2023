import requests
import os, sys
sys.path.insert(0, os.path.abspath(".."))
from modules.LSM303 import Compass
from CommandScripts import AutoHelp
from simple_pid import PID
import time
import numpy as np
import math

class VFH_obstacle_avoidance:
    def __init__(self, distance_threshold, general_angle):
        self.distance_threshold = distance_threshold # any obstacle greater than threshold is considered too far away and won't be taken into account
        self.general_angle = general_angle
        
    def get_target_angle(self, lidar_data, rover_angle, current_longlat, target_longlat):
        """Obstacle avoidance logic that uses vector field histogram (VFH)
        PARAMS:
            lidar_data (list):      array of 360 elements. each index is an angle
            rover_angle (int):      the angle that the rover is currently facing      
            current_longlat(tuple): (longitude, latitude) of rover's current position
            target_longlat (tuple): (longitude, latitude) of rover's target position
        RETURNS:
            integer. The angle we want to turn to
        """

        '''
        changes:
            * added a parameter that has the general angle that we're trying to move to
                - we should always be trying to move towards this angle
            * uses get_bearing() method to get target angle
        TODO:
            * rename file to RoverNavigation
        '''

        # Parameters
        sector_size = 45 # Angular sector size in degrees

        # Initialize histogram bins
        num_bins = int(360 / sector_size)  
        histogram = np.zeros(num_bins, dtype=int)
        
        # Populate histogram with obstacle data
        for angle, distance in enumerate(lidar_data):
            bin_index = int(angle / sector_size)
            histogram[bin_index] += 0 if distance < self.distance_threshold else 1
        
        # print(f"{histogram=}")
        # Find the direction with the least obstacle density (peak)
        smoothed_histogram = np.convolve(histogram, np.ones(3), mode='same')  # Smooth histogram
        # print(f"{smoothed_histogram=}")
        target_direction = np.argmax(smoothed_histogram) # returns the index of the element with the highest value
        
        # Calculate the target angle (angle of the selected direction)
        target_angle = target_direction * sector_size
        # print(f"Product of target_direction and sector_size: {target_angle=}")
        
        # Incorporate the general target angle (placeholder)
        general_angle = self.get_bearing(current_longlat, target_longlat)
        
        if general_angle is not None:
            # Blend the obstacle avoidance direction with the general target angle
            target_angle = 0.7 * target_angle + 0.3 * general_angle
            
        # Ensure that target_angle is within 0-360 degrees
        target_angle %= 360
        
        # Calculate the rotation needed based on compass and lidar data
        lidar_angle_start = 0  # Assume lidar starts at the first angle in data
        angle_difference = (rover_angle - lidar_angle_start) % 360  # Adjust angle difference to be within 0-360
        rotation_angle = (target_angle - angle_difference) % 360  # Adjust rotation angle to be within 0-360
        # print(f"{rotation_angle=}")
        
        # return rotation_angle
        # Return the rotation angle for the robot to align with the clear direction
        return lidar_data.index(max(lidar_data[rotation_angle:rotation_angle+sector_size+1]))
    
    def get_bearing(self, current_longlat, target_longlat):
        # Calculate the bearing from current position to target position
        lat1, lon1 = current_longlat
        lat2, lon2 = target_longlat

        delta_lon = lon2 - lon1 #angular distance between both positions

        #uses Haversine formula to calculate bearing angle from one pos to target
        y = math.sin(delta_lon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)

        bearing = math.degrees(math.atan2(y, x))
        
        # Normalize the bearing to the range 0-360 degrees
        bearing = (bearing + 360) % 360
        return bearing 

class GPS_Nav:
    def __init__(self, max_speed, max_steering, GPS, compass, GPS_coordinate_map):
        self.max_speed = max_speed
        self.max_steering = max_steering
        self.commands = [0,1,0,'D',0,0]
        self.compass = compass
        self.AutoHelp = AutoHelp.AutoHelp()

        self.GPS = GPS
        self.GPS_coordinate_map = GPS_coordinate_map
        self.GPS_target = self.GPS_coordinate_map[0]

        # Create a PID controller for steering
        self.steer_controller = PID(Kp=1, Ki=0.5, Kd=0.05, setpoint=0)
        self.steer_controller.sample_time = 0.1
        self.steer_controller.output_limits = (-self.max_steering, self.max_steering)
        self.steer_controller.proportional_on_measurement = True  # Use derivative of error instead of error for Kp

        # Create a PID controller for speed
        self.speed_controller = PID(Kp=0.5, Ki=0.1, Kd=0.05, setpoint=self.max_speed)
        self.speed_controller.sample_time = 0.1
        self.speed_controller.output_limits = (0, self.max_speed)

    


    def PID_steer(self, commands, steer_output, angle):
        speed_error = self.max_steering/abs(steer_output)   # scale speed error based on steering output
        print("Speed Error:", speed_error)
        speed_output = self.speed_controller(speed_error)
        self.commands[4] = round(speed_output)
        if angle == "right":
            self.commands[5] = abs(round(steer_output))
        elif angle == "left":
            self.commands[5] = -abs(round(steer_output))
        return self.AutoHelp.jsonify_commands(commands)


    def forward_drive(self, commands):
        self.commands[4] = self.max_speed
        self.commands[5] = 0
        return self.AutoHelp.jsonify_commands(commands)

    def spin(self, commands, angle):
        self.commands[5] = 0
        if angle == "right":
            self.commands[4] = abs(round(self.max_speed/2))
        elif angle == "left":
            self.commands[4] = -abs(round(self.max_speed/2))
        
        return self.AutoHelp.jsonify_commands(commands)

    def stop_rover(self, commands):
        self.commands = [0, 0, 0, 'D', 0, 0]
        return self.AutoHelp.jsonify_commands(commands)

    def goto_next_coordinate(self):
        try:
            self.GPS_coordinate_map.pop(0)
            self.GPS_target = self.GPS_coordinate_map[0]
            print("Going to new coordinate!")
            print("NEW:", self.GPS_target)
        except:
            print("No more GPS coordniates in Mission... Mission Success!")
            exit(1)


    def change_modes(self, desired_mode):
        if desired_mode == 'D':
            self.commands[3] = 'D'
            self.commands[4] = 0
            self.commands[5] = 0
            return self.AutoHelp.jsonify_commands(self.commands)
        elif desired_mode == 'S':
            self.commands[3] = 'S'
            self.commands[4] = 0
            self.commands[5] = 0
            return self.AutoHelp.jsonify_commands(self.commands)
        elif desired_mode == 'T':
            self.commands[3] = 'T'
            self.commands[4] = 0
            self.commands[5] = 0
            return self.AutoHelp.jsonify_commands(self.commands)


    def get_steering(self, current_GPS, GPS_target):
        rover_heading = self.compass.get_heading()
        bearing = self.AutoHelp.get_bearing(current_GPS, GPS_target)
        self.steer_controller.setpoint= bearing
        distance = round(self.AutoHelp.get_distance(current_GPS, GPS_target)[0]*1000, 3)
        direction = round((bearing - rover_heading + 360) % 360, 3)

        steer_error = self.steer_controller(rover_heading)
        print("Direction:", direction)

        if distance <= 3:
            print("Arrived at target!")
            self.goto_next_coordinate()
            time.sleep(3)
            return self.stop_rover(self.commands)

        if abs(direction) > 15:

            if direction >= 150 and direction <= 180:
                print("Going to Spin Mode Right")
                self.change_modes('S')
                return self.spin(self.commands, 'right')
            elif direction >= 180 and direction <= 210:
                print("Going to Spin Mode Left")
                self.change_modes('S')
                return self.spin(self.commands, 'left')

            if self.commands[3] == 'S' and direction < 30 or direction > 330:
                self.change_modes('D')

            if self.commands[3] == 'D' and direction < 150:
                print("Turning right")
                return self.PID_steer(self.commands, steer_error, 'right')
            elif self.commands[3] == 'D' and direction > 210:
                print("Turning left")
                return self.PID_steer(self.commands, steer_error, 'left')
        rover_heading = bearing
        
        if direction <= 15:
            print("Moving forward")
            self.change_modes('D')
            return self.forward_drive(self.commands)
