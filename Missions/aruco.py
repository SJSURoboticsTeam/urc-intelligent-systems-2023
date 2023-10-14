import time
import sys, os
sys.path.insert(0, os.path.abspath(".."))
from CommandScripts.GPS_NAV import GPS_Nav
# Define a list of GPS coordinates for the rover to navigate to
gps_coordinates = [
    (37.33, -121.88),  # Example coordinates, replace with actual waypoints
    (37.34, -121.89),
    # Add more coordinates as needed
]

# Create an instance of the GPS_Nav class
rover = GPS_Nav(max_speed=5, max_steering=30, GPS=None, compass=None, GPS_coordinate_map=gps_coordinates)

# Iterate through the GPS coordinates
for target_gps in gps_coordinates:
    print(f"Navigating to GPS coordinates: {target_gps}")

    # Initialize variables for Aruco tag detection
    aruco_detected = False
    max_attempts = 5  # Number of attempts to search for Aruco tag
    attempts = 0

    while not aruco_detected and attempts < max_attempts:
        # Get current GPS coordinates and update steering behavior
        current_gps = rover.get_current_gps_coordinates()

        # Calculate bearing and steering to the target GPS coordinates
        rover_commands = rover.get_steering(current_gps, target_gps)

        # Execute the rover's movement based on the computed commands
        rover.execute_commands(rover_commands)

        # Check if an Aruco tag has been detected
        aruco_detected = rover.check_aruco_detection()

        if not aruco_detected:
            print("Aruco tag not detected. Spinning to search...")
            rover.spin_in_place()

        attempts += 1

    if aruco_detected:
        print("Aruco tag detected! Stopping and proceeding to the next GPS coordinate.")
        rover.stop_rover()

    else:
        print("Aruco tag not found after multiple attempts. Proceeding to the next GPS coordinate.")

    # Wait for a moment before moving to the next GPS coordinate (adjust as needed)
    time.sleep(2)

print("Mission completed.")
