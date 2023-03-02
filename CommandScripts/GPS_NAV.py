import requests
import os, sys
import time
sys.path.insert(0, os.path.abspath(".."))
from modules.LSM303 import Compass
from modules.OAKD import OakD
from modules.ArucoTagDetector import ArucoTagAutonomy
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
        self.speed_controller = PID(Kp=0.5, Ki=0.1, Kd=0.05, setpoint=self.max_speed)
        self.speed_controller.sample_time = 0.1
        self.speed_controller.output_limits = (0, self.max_speed)

        ############################
        # Aruco Tag Detection stuff
        self.aruco_autonomy = ArucoTagAutonomy(cap=OakD())

        # we use these variables to occasionally re-check the location of the aruco tag or gate because our initial detection may be slightly inaccurate
        self.navigating_to_tag = False
        self.distance_until_recheck = -1 # TODO: checking the location of the tag mid-journey is not implemented yet

        # when we search for the aruco tag, we basically need to spin in a complete circle, coming to a complete stop to search for it every 30 degrees
        self.spinning_and_searching = False
        self.camera_stabilized = False # the aruco tag detection algorithm is sensitive to movement, so we need to stop the rover before we search for the tag
        self.starting_heading = 0
        self.num_failed_searches = 0 # if we can't find the tag after 12 searches, (a whole rotation of the rover), then we need to stop searching for the tag
        self.SEARCH_ANGLE = 30


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

    def spin_and_search(self, current_GPS, rover_heading):
        """
        This function is called when we need to search for an aruco tag on a post.
        It requires self.starting_heading to be set to the rover's heading when it starts searching.
        It will spin the rover in a complete circle, stopping to search for the aruco tag every 30 degrees.
        If it finds the aruco tag, it will set self.GPS_target to the GPS coordinates of the post and return a stop command.
        If it doesn't find the aruco tag, it will return a spin left command.
        """

        if self.camera_stabilized: # we can only search for aruco tags when the rover is stopped
            tvec = self.aruco_autonomy.search_for_post()

            if tvec is not None: # if we've found a post, we can set the GPS target to the post's GPS coordinates
                print(f"!!!Found Aruco Post {self.aruco_autonomy.target_tags}!!!")
                self.GPS_target = ArucoTagAutonomy.translate_lat_lon(lat=current_GPS[1], lon=current_GPS[0], tvec=tvec, heading=rover_heading)
                # also need to stop spinning and searching and start navigating to the aruco tag
                self.spinning_and_searching = False
                self.navigating_to_tag = True

                return self.stop_rover(self.commands) # we should already be stopped, but we have to return something, so we'll just return a stop command

            else: # otherwise, we need to keep spinning and searching
                self.num_failed_searches += 1
                self.camera_stabilized = False
                self.change_modes('S')
                return self.spin(self.commands, 'left')

        elif self.num_failed_searches == 12: # if we've searched for the aruco tag 12 times (completing a full rotation) and haven't found it, there is probably no aruco tag so we can stop searching
            self.spinning_and_searching = False
            self.change_modes('D')
            return self.stop_rover(self.commands)

        else:
            # compute the difference between the current angle and the heading taking into account the wrap around
            angle_diff = (rover_heading - self.starting_heading + 180) % 360 - 180  # https://stackoverflow.com/a/7869457

            if abs(angle_diff) >= self.SEARCH_ANGLE * self.num_failed_searches: # if we've rotated at least 30 degrees, we can stop and search for the aruco tag
                self.camera_stabilized = True
                self.change_modes('D')
                return self.stop_rover(self.commands)
            else: # otherwise, just keep spinning
                self.change_modes('S')
                return self.spin(self.commands, 'left')

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

        if self.spinning_and_searching:
            return self.spin_and_search(current_GPS, rover_heading)

        bearing = self.AutoHelp.get_bearing(current_GPS, GPS_target)
        self.steer_controller.setpoint= bearing
        distance = round(self.AutoHelp.get_distance(current_GPS, GPS_target)[0]*1000, 3)
        direction = round((bearing - rover_heading + 360) % 360, 3)

        steer_error = self.steer_controller(rover_heading)
        print("Direction:", direction)

        # if we're within 2 meters of the aruco tag, we can go to the next coordinate.
        if distance <= 2 and self.navigating_to_tag:
            # TODO: we will have to modify this for the gate, also we should probably double check to make sure we're actually at the aruco tag
            print(f"Arrived at Aruco Post {self.aruco_autonomy.target_tags}")

            # after printing that we reached the post, update the target tag and reset all aruco-related variables
            self.reset_aruco_variables(update_target_tag=True)

            self.goto_next_coordinate()
            time.sleep(3)
            return self.stop_rover(self.commands)

        # if we're within 3 meters of the GPS coordinate, but we haven't started looking for an aruco tag, start looking
        elif distance <= 3 and self.num_failed_searches == 0 and not self.navigating_to_tag:
            self.starting_heading = bearing
            self.spinning_and_searching = True
            self.camera_stabilized = True

            return self.stop_rover(self.commands)  # stop the rover so that the movement doesn't interfere with the aruco tag detection algorithm

        # lastly, if we're within 3 meters of the GPS coordinate, and we've already searched for the aruco tag but didn't find it, we can assume there is none
        # nearby, so we can just go to the next coordinates
        elif distance <= 3 and self.num_failed_searches == 12 and not self.navigating_to_tag:
            self.reset_aruco_variables()

            print("Arrived at GPS target!")
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
                return self.PID_steer(self.commands, direction, 'left')

        else:
            print("Moving forward")
            self.change_modes('D')
            return self.forward_drive(self.commands)

        # FOR GATE: navigate close to the gate (perhaps perpendicular to it) and then spin to face the center between the two posts. Then, transfer into translate mode and go forward.


    def reset_aruco_variables(self, update_target_tag=False):
        if update_target_tag:
            self.aruco_autonomy.update_target_tag()

        self.navigating_to_tag = False
        self.num_failed_searches = 0
        self.distance_until_recheck = -1
        self.camera_stabilized = False
        self.spinning_and_searching = False
