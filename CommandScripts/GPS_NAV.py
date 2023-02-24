import requests
import os, sys
sys.path.insert(0, os.path.abspath(".."))
from modules.LSM303 import Compass
from CommandScripts import AutoHelp
from simple_pid import PID

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
        self.speed_controller = PID(Kp=0.5, Ki=0.1, Kd=0.05, setpoint=0)
        self.speed_controller.sample_time = 0.1
        self.speed_controller.output_limits = (0, self.max_speed)
        

    def PID_steer(self, commands, steer_error, angle):
        steer_output = self.steer_controller(steer_error)
        speed_output = self.speed_controller(steer_output)
        self.commands[4] = round(speed_output)
        if angle == "right":
            self.commands[5] = abs(round(steer_output))
        elif angle == "left":
            self.commands[5] = round(steer_output)
        return self.AutoHelp.jsonify_commands(commands)

    def drive(self, commands, drive_error):
        speed_output = self.speed_controller(drive_error)
        self.commands[4] = speed_output
        self.commands[5] = 0
        return self.AutoHelp.jsonify_commands(commands)

    def spin(self, commands, spin_error):
        self.commands[4] = round(self.max_speed/2)
        self.commands[5] = 0
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
        distance = round(self.AutoHelp.get_distance(current_GPS, GPS_target)[0]*1000, 3)
        direction = round((bearing - rover_heading + 360) % 360, 3)
        # direction = 
        print("Direction:", direction)


        if abs(direction) > 5:

            if direction > 90 and direction < 150:
                print("Going to Spin Mode")
                self.change_modes('S')
                return self.spin(self.commands, direction)
            
            if self.commands[3] == 'S' and direction < 120:
                self.change_modes('D')

            if direction < 120:
                print("Turning right")
                return self.PID_steer(self.commands, direction, 'right')
            else:
                print("Turning left")
                return self.PID_steer(self.commands, direction, 'left')
        rover_heading = bearing
        
        if distance > 0.5:
            print("Moving forward")
            return self.drive(self.commands, direction)
        else:
            print("Arrived at target!")
            self.goto_next_coordinate()
            return self.stop_rover(self.commands)