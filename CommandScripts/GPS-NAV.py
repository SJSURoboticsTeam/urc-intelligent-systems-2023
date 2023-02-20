import requests
import json

class GPSNav:
    def __init__(self, url, max_speed, max_steering, GPS, GPS_coordinate_map):
        self.url = url
        self.max_speed = max_speed
        self.max_steering = max_steering
        self.commands = [0,0,0,'D',0,0]
        self.current_GPS = [0,0]
        self.GPS = GPS
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