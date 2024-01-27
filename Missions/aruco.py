import time
import sys, os
sys.path.insert(0, os.path.abspath(".."))
from Autonomous_Systems.RoverNavigation import RoverNaviagtion
from Autonomous_Systems.AutoHelp import AutoHelp
from modules.GPS import gpsRead
from modules.WiFi import WiFi
from modules.LSM303 import Compass
from modules.ArucoTagDetector import ArucoTagAutonomy

# Define a list of GPS coordinates for the rover to navigate to
gps_coordinates = [
    (37.336924389574556, -121.88181061869382),  # Example coordinates, replace with actual waypoints
    (37.33685774647711, -121.88176166838032),
    # Add more coordinates as needed
]

# Create an instance of the GPS_Nav class
rover = RoverNaviagtion(max_speed=5, max_steering=30, GPS=None, compass=None, GPS_coordinate_map=gps_coordinates)

# Iterate through the GPS coordinates
for target_gps in gps_coordinates:
    print(f"Navigating to GPS coordinates: {target_gps}")

    # Initialize variables for Aruco tag detection
    aruco_detected = False
    max_attempts = 5  # Number of attempts to search for Aruco tag
    attempts = 0

    while not aruco_detected and attempts < max_attempts:
        # Get current GPS coordinates and update steering behavior
        current_gps = gpsRead.get_position()

        # Calculate bearing and steering to the target GPS coordinates
        rover_commands = rover.get_steering(current_gps, target_gps)
        
        #WIFI PROCEDURE
        print("Current GPS:", current_gps)
        print("Target GPS:", target_gps)
        print("Bearing:", round(AutoHelp.get_bearing(current_gps, target_gps), 3))
        print("distance:", round(AutoHelp.get_distance(current_gps, target_gps)[0]*1000, 3))
        print("Sending Command:", rover_commands)

        # Execute the rover's movement based on the computed commands
        WiFi.write_data(rover_commands)

        # Check if an Aruco tag has been detected   IN BOOLEAN
        aruco_detected = (ArucoTagAutonomy.search_for_tags() == [], [])

        if not aruco_detected:
            print("Aruco tag not detected. Spinning to search...")
            rover.spin_and_search(gpsRead.get_position(), Compass.get_heading())

        attempts += 1

    if aruco_detected:
        print("Aruco tag detected! Stopping and proceeding to the next GPS coordinate.")
        rover.stop_rover()

    else:
        print("Aruco tag not found after multiple attempts. Proceeding to the next GPS coordinate.")

    # Wait for a moment before moving to the next GPS coordinate (adjust as needed)
    time.sleep(2)
 
print("Mission completed.")
