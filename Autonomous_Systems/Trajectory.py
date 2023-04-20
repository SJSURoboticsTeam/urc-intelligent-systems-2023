import math

class Trajectory:
    def __init__(self, camera_data, Lidar_data):
        self.camera_data = camera_data
        self.Lidar_data = Lidar_data
    
    def object_detected(self, d):
            angle = math.atan(d, d)
            self.commands[5] = angle
            trajectory_commands = self.jsonify_commands(self.commands)
            return trajectory_commands
    

    def detect_obstacle(self):
        # reduce the memory of all the obstacles currently seen (anything with a counter less than -1)
        self.map[self.map < -1] += 1

        # now, simulate checking for obstacles in the 8 directions around the rover
        # this line gets a subarray of the map that is 3x3 (when lidar range is 1, 5x5 for lidar range 2, etc)
        # centered on the rover, using max and min to make sure we don't go out of bounds

        nearby_map = self.map[max(self.rover_y - self.lidar_range, 0):min(self.rover_y +  self.lidar_range + 1, self.map_height),
                                max(self.rover_x - self.lidar_range, 0):min(self.rover_x +  self.lidar_range + 1, self.map_width)]
        nearby_map[nearby_map <= -1] = -1 - self.obstacle_memory