import math
import np


class LiVFH:
    def __init__(self, robot_radius, safety_distance, turn_radius, measurement_threshold, wide_opening_deg=10):
        self.robot_radius = robot_radius
        self.safety_distance = safety_distance
        self.turn_radius = turn_radius
        self.threshold = measurement_threshold
        self.current_direction = 0
        self.former_direction = 0
        self.mu1 = 5
        self.mu2 = 2
        self.mu3 = 2
        self.robot_radius_safety_distance = self.robot_radius + self.safety_distance
        self.s_max = wide_opening_deg
        self.current_direction = 0
        self.former_direction = 0
        

    def read_laser_scan_data(self, file_path):
        with open(file_path, 'r') as file:
            laser_data = [float(value) for line in file for value in line.split()]
        print(laser_data)
        return laser_data




    
    def calculate_d2(self, lidar_scan: list) -> list:
        # Calculates the distance from turncircle to scanned object
        # Parameters:
        # Lidar_scan (list): Thresholded lidar_scan
        # Returns:
        # List of distances to scanned objects
        distances = []
        for indx in range(len(lidar_scan)):
            if indx <=90:
                angle_to_obj = 90 - indx
            elif indx > 90 and indx <= 180:
                angle_to_obj = 180 - indx
            elif indx > 180 and indx <= 270:
                angle_to_obj = 270 - indx
            else:
                angle_to_obj = 360 - indx

            if lidar_scan[indx] != 0:
                distances.append(math.floor(math.sqrt(self.turn_radius**2 + lidar_scan[indx]**2 - 2 * self.turn_radius*lidar_scan[indx] * math.cos(angle_to_obj))))
            else:
                distances.append(np.inf)

        return distances

    def create_bin_histogram(self, lidar_scan: list) -> list:
        # """Creates a binary histgram
        # Parameters:
        # lidar_scan (list): Thresholded lidar_scan
        # Returns:
        # Binary histogram
        # """
        bin_hist = []

        for scan in lidar_scan:
            if scan != 0:
                bin_hist.append(1)
            else:
                bin_hist.append(0)

        edge_hist = np.diff(bin_hist, 1) #Find edges in lidar scan
        edge_hist = list(edge_hist)
        edge_hist.insert(0, 0)

        
        for indx in range(len(bin_hist)):
            if edge_hist[indx] != 0:
                if edge_hist[indx] > 0:
                    angle_to_expand_by = self.enlarge_obstacle(lidar_scan[indx])
                    for index in range( indx - angle_to_expand_by, indx):
                            bin_hist[index] = 1
                else:
                    angle_to_expand_by = self.enlarge_obstacle(lidar_scan[indx - 1])
                    if (indx + angle_to_expand_by) >= len(bin_hist):
                        rest = indx + angle_to_expand_by - len(bin_hist)
                        for index in range(indx, len(bin_hist)):
                            bin_hist[index] = 1
                        for index in range(rest):
                            bin_hist[index] = 1
                    else:
                        for index in range(indx, indx + angle_to_expand_by):
                            bin_hist[index] = 1
        return bin_hist
    
    def enlarge_obstacle(self, distance) -> int:
        # """Returns the amount the binary histogram needs to expand the scanned points by
        # Parameters:
        # distance (int): A single lidar_scan index, so a distance is given
        # Returns:
        # The distance in centimeters
        # """
        angle_to_expand_by = round(((math.atan(self.robot_radius+self.safety_distance/distance)/2)/math.pi)*180)
        return angle_to_expand_by

    def calculate_is_blocked_right(self, lidar_scan: list) -> int:
        # """Let's the system know if pathing to the right side of the robot is blocked
        # Parameters:
        # lidar_scan: list: Thresholded lidar_scan
        # Returns:
        # The right hand degree from where the histogram will be masked 
        # """
        phi_r = 180
        distances = self.calculate_d2(lidar_scan=lidar_scan)
        for index in range(359, 180):
            if distances[index] < (self.turn_radius + (self.robot_radius + self.safety_distance)+1):
                phi_r = index
                return phi_r
        return phi_r

    def calculate_is_blocked_left(self, lidar_scan: list) -> int:
        # """Let's the system know if pathing to the left side of the robot is blocked
        # Parameters:
        # lidar_scan: list: Thresholded lidar_scan
        # Returns:
        # The left hand degree from where the histogram will be masked 
        # """
        phi_l = 180
        distances = self.calculate_d2(lidar_scan=lidar_scan)
        for index in range(0, 360):
            if distances[index] < (self.turn_radius + (self.robot_radius + self.safety_distance)+1):
                phi_l = index
                return phi_l
        return phi_l

    def create_masked_histogram(self, binary_histogram: list, lidar_scan: list) -> list:
        # """Takes the binary histogram, and masks it, so only the reachable directions, are considered
        # Parameters:
        # binary_hisogram (list): A binary histogram
        # lidar_scan: list: Thresholded lidar_scan
        # Returns:
        # A masked binary histogram
        # """
        phi_l = self.calculate_is_blocked_left(lidar_scan)
        phi_r = self.calculate_is_blocked_right(lidar_scan)

        for index in range(len(binary_histogram)):
            if binary_histogram[index] == 0 and ((index >= 0 and index <= phi_l) or (index <= 359 and index >= phi_r)):
                pass
            else:
                binary_histogram[index] = 1
        return binary_histogram

    def find_candidates(self, masked_histogram: list, kt: int) -> list:
        # """Finds the possible driving headings for the shortest distances around obstacles
        # Parameters:
        # masked_histogram: list: A masked histogram
        # kt: int: The direction of the goal
        # Returns:
        # A list of possible headings to drive (in degrees)
        # """
        candidates_kr = []
        candidates_kl = []
        resulting_candidates = []
        wrap_around_kl = int
        is_wrapped_around = False

        padded_histogram = masked_histogram
        padded_histogram.insert(0, padded_histogram[-1])

        edge_histogram = np.diff(padded_histogram, 1)

        for index in range(len(edge_histogram)):
            if edge_histogram[index] == -1:
                candidates_kr.append(index)
            elif edge_histogram[index] == 1:
                if len(candidates_kr) == 0:
                    wrap_around_kl = index
                    is_wrapped_around = True
                else:
                    candidates_kl.append(index)

        if is_wrapped_around == True:
            candidates_kl.append(wrap_around_kl)
        
        for index in range(len(candidates_kl)):
            if (abs(candidates_kr[index] - candidates_kl[index])) <= self.s_max: # if opening is narrow
                resulting_candidates.append(int((candidates_kl[index] + candidates_kr[index])/2))
            else:
                cr = candidates_kr[index] + (self.s_max/2)
                if cr < 0:
                    cr = cr + 360
                cl = candidates_kl[index] - (self.s_max/2)
                if cl > 360:
                    cl = cl - 360
                resulting_candidates.append(int(cr))
                resulting_candidates.append(int(cl))
                if (kt >= cr and kt <= cl):
                    resulting_candidates.append(int(kt))
        if (is_wrapped_around == True and (kt >= resulting_candidates[-2] or kt <= resulting_candidates[-1])):
            resulting_candidates.append(int(kt))
            

        if len(resulting_candidates) == 0:
            resulting_candidates.append(kt)
        return resulting_candidates

    def cost_function(self, candidates: list, kt: int, current_direction: int, former_direction: int) -> list:
        # """Grades the candidates, so that the lowest score is the best candidate. 
        # Parameters:
        # candidates (list): A list of possible turning directions
        # kt (int): The directions towards the goal in degrees
        # current_direction (int): The direction the robot is travling at the moment in degrees
        # former_direction (int): The direction the robot traveled last itteration in degrees
        # Returns:
        # A list of the scores, the individual candidates got, so that the indexes matches each other
        # """
        scores = []
        delta_c_kt = int
        delta_c_current_direction = int
        delta_c_former_direction = int

        for index in range(len(candidates)):
            if abs(candidates[index] - kt) > 180:
                delta_c_kt = 360 - abs(candidates[index] - kt)
            else:
                delta_c_kt = abs(candidates[index] - kt)

            if abs(candidates[index] - current_direction) > 180:
                delta_c_current_direction = 360 - abs(candidates[index] - current_direction)
            else:
                delta_c_current_direction = abs(candidates[index] - current_direction)
            
            if abs(candidates[index] - former_direction) > 180:
                delta_c_former_direction = 360 - abs(candidates[index] - former_direction)
            else:
                delta_c_former_direction = abs(candidates[index] - former_direction)

            score = self.mu1 * delta_c_kt + self.mu2 * delta_c_current_direction + self.mu3 * delta_c_former_direction
            scores.append(score)
            
        return scores

    def find_steering_direction(self, lidar_scan: list, target_direction: int):
        """Finds the direction the robot needs to turn, to avoid obstacles
        Parameters:
        lidar_scan (list): The raw lidar scan from the 360 degree lidar
        target_direction (int): The direction the robot is traveling, in relationship to itself, in degrees
        Returns:
        The angle the robot needs to turn, in order to not hit an obstacle in radians
        """

        #Remove any object the LiDAR detected that aren't within the threshold, and convert scan into centimeters
        thresholded_scan = list()
        for scan in lidar_scan:
            if scan == np.inf:
                thresholded_scan.append(2000)
            else:
                thresholded_scan.append(math.floor(scan * 100))
        
        for indx in range(len(thresholded_scan)):
            if thresholded_scan[indx] > self.threshold:
                thresholded_scan[indx] = 0

        #Create a positiv target direction, so that candidates can be picked out
        if target_direction < 0:
            target_direction_positive = target_direction + 360
        else:
            target_direction_positive = target_direction


        binary_histogram = self.create_bin_histogram(thresholded_scan) # Creates the binary histgram that needs to be masked
        masked_histogram = self.create_masked_histogram(binary_histogram=binary_histogram, lidar_scan=thresholded_scan) # Masks the binary histogram
        turning_candidates = self.find_candidates(masked_histogram=masked_histogram, kt=target_direction_positive) # Finds the candidates needed to make sure no collision with an obstacle is made

        #Makes sure that turning candidates that are over 180 degrees is negative, so the robot can turn right
        for index in range(len(turning_candidates)):
            if turning_candidates[index] > 180:
                turning_candidates[index] = turning_candidates[index] - 360

        # if TEST:
        #     binary_hist_msg = LaserScan()
        #     candidates_msg = LaserScan()
        #     hist_pub = rospy.Publisher("/obstacle/histogram", LaserScan, queue_size=1024)
        #     candidate_pub = rospy.Publisher("/obstacle/candidates", LaserScan, queue_size=1024)

        #     print(turning_candidates)
        #     candidates_hist = list(np.zeros(len(binary_histogram)))
        #     for candidate in turning_candidates:
        #         candidates_hist[candidate] = 1

        #     candidates_msg.ranges = candidates_hist
        #     binary_hist_msg.ranges = masked_histogram
            
        #     hist_pub.publish(binary_hist_msg)
        #     candidate_pub.publish(candidates_msg)

            

        #Scores the candidates, so the best candidate has the lowest score
        scores = self.cost_function(candidates=turning_candidates, kt=target_direction, current_direction=self.current_direction, former_direction=self.former_direction)

        #Finds the lowest score, and relates finds the index. That index denotes the the index that the best candidate has.
        best_score = min(scores)
        direction_to_turn = turning_candidates[scores.index(best_score)]

        #Sets current and former direction
        self.former_direction = self.current_direction
        self.current_direction = direction_to_turn

        #Makes direction_to_turn negative if it's over 180
        if direction_to_turn > 180:
            direction_to_turn - 360

        return direction_to_turn * (math.pi/180) # Returns direction in radians

   

if __name__ == "__main__":
    # Create an instance of LiVFH
    robot_radius = 10  # Replace with your robot's radius in centimeters
    safety_distance = 20  # Replace with your desired safety distance in centimeters
    turn_radius = 30  # Replace with your desired turn radius in centimeters
    measurement_threshold = 100  # Replace with your measurement threshold in centimeters

    obstacle_avoidance = LiVFH(robot_radius, safety_distance, turn_radius, measurement_threshold)

    # Read laser scan data from the file
    laser_scan_data = obstacle_avoidance.read_laser_scan_data('laser_scan_data.txt')

    # Set a target direction (in degrees)
    target_direction_deg = 45  # Replace with your desired target direction in degrees

    # Find the direction to turn for obstacle avoidance (in radians)
    direction_to_turn_rad = obstacle_avoidance.find_steering_direction(laser_scan_data, target_direction_deg)

    print("Direction to Turn:", math.degrees(direction_to_turn_rad), "degrees")
