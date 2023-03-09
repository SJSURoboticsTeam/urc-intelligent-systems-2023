import requests
import os, sys
sys.path.insert(0, os.path.abspath(".."))
from Autonomous_Systems import AutoHelp
from simple_pid import PID
import time
import numpy as np
from math import atan2, sqrt, pi
from numpy import linalg as la


class KalmanFilter:
    def __init__(self, dt):
        self.dt = dt
        self.A = np.array([[1, 0, dt, 0],
                           [0, 1, 0, dt],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])
        self.C = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])
        self.Q = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 0.1, 0],
                           [0, 0, 0, 0.1]])
        self.R = np.array([[10, 0],
                           [0, 10]])
        self.P = np.eye(self.A.shape[0]) * 1000

    def predict(self, x):
        x = np.dot(self.A, x)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return x

    def update(self, x, z):
        y = z - np.dot(self.C, x)
        S = np.dot(np.dot(self.C, self.P), self.C.T) + self.R
        K = np.dot(np.dot(self.P, self.C.T), la.inv(S))
        x = x + np.dot(K, y)
        self.P = np.dot((np.eye(self.P.shape[0]) - np.dot(K, self.C)), self.P)
        return x

class GridMap:
    def __init__(self, width, height, resolution):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.map = np.zeros((width, height), dtype=int)

    def update(self, x, y):
        i = int(x / self.resolution)
        j = int(y / self.resolution)
        if i >= 0 and i < self.width and j >= 0 and j < self.height:
            self.map[i, j] += 1

    def get_map(self):
        return self.map




class GPS_Nav:
    def __init__(self, max_speed, max_steering, GPS, IMU, GPS_coordinate_map):
        self.max_speed = max_speed
        self.max_steering = max_steering
        self.commands = [0,1,0,'D',0,0]
        self.IMU = IMU
        self.AutoHelp = AutoHelp.AutoHelp()

        self.GPS = GPS
        self.GPS_coordinate_map = GPS_coordinate_map
        self.GPS_target = self.GPS_coordinate_map[0]

        self.filter = KalmanFilter(dt=0.1)
        self.position = np.array([0, 0, 0, 0])  # x, y, vx, vy
        self.map = GridMap(width=100, height=100, resolution=0.5)
        # self.path = [(10, 20), (30, 40), (50, 60)]
        self.path = [(10, 20), (30, 40), (20, 50), (60, 80), (90, 70), (100, 90)]

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


    def follow_path(self, goal):
        # Compute heading to goal
        goal_x, goal_y = goal
        dx = goal_x * self.map.resolution - self.position[0]
        dy = goal_y * self.map.resolution - self.position[1]
        heading = atan2(dy, dx) * 180.0 / pi
        if heading > 180.0:
            heading -= 360.0
        elif heading < -180.0:
            heading += 360.0

        # Compute distance to goal
        dist = sqrt(dx ** 2 + dy ** 2)

        # Kalman filter predict and update
        self.position = self.filter.predict(self.position)
        self.position = self.filter.update(self.position, np.array([goal_x, goal_y]))

        # Update the grid map
        self.map.update(self.position[0], self.position[1])

        # Compute forward and angular velocities
        k_v = 0.1  # Velocity gain
        k_w = 0.5  # Heading gain
        quat_i, quat_j, quat_k, quat_real = self.IMU.get_rotation()
        rover_heading = self.IMU.get_heading(quat_real, quat_i, quat_j, quat_k)
        v = k_v * dist
        w = k_w * (heading - rover_heading)

        # Send commands to Rover to follow path
        if dist <= 1:
            self.path.pop(0)  # Remove the current goal from the path
            if len(self.path) > 0:
                goal_x, goal_y = self.path[0]  # Set the next goal as the new target
                dx = goal_x * self.map.resolution - self.position[0]
                dy = goal_y * self.map.resolution - self.position[1]
                heading = atan2(dy, dx) * 180.0 / pi
                if heading > 180.0:
                    heading -= 360.0
                elif heading < -180.0:
                    heading += 360.0
                dist = sqrt(dx ** 2 + dy ** 2)
                w = k_w * (heading - rover_heading)  # Recompute angular velocity
            else:
                print("No more goals in path")
                return self.stop_rover(self.commands)

        # Determine whether to turn left or right
        if w > 0:
            print("Turning right")
            turn_direction = 'right'
        else:
            print("Turning left")
            turn_direction = 'left'

        # Convert angular velocity to a steering output
        steer_output = w / self.max_steering

        # Send steering commands to the Rover
        return self.PID_steer(self.commands, steer_output, turn_direction)






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
        quat_i, quat_j, quat_k, quat_real = self.IMU.get_rotation()
        rover_heading = self.IMU.get_heading(quat_real, quat_i, quat_j, quat_k)
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
