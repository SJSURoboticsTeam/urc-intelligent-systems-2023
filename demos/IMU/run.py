import sys
sys.path.append( '../../')
from modules.BN08x import BN08x
import time
import math

# Initialize the BN08x sensor
sensor = BN08x()

while True:
    # Get acceleration values
    accel_x, accel_y, accel_z = sensor.get_acceleration()
    print("Acceleration: ({:.2f}, {:.2f}, {:.2f}) m/s^2".format(accel_x, accel_y, accel_z))

    # Get gyro values
    gyro_x, gyro_y, gyro_z = sensor.get_gyro()
    print("Gyro: ({:.2f}, {:.2f}, {:.2f}) deg/s".format(gyro_x, gyro_y, gyro_z))

    # Get magnetometer values
    mag_x, mag_y, mag_z = sensor.get_mag()
    print("Magnetometer: ({:.2f}, {:.2f}, {:.2f}) microteslas".format(mag_x, mag_y, mag_z))

    # Get rotation values
    quat_i, quat_j, quat_k, quat_real = sensor.get_rotation()
    print("Rotation: ({:.2f}, {:.2f}, {:.2f}, {:.2f}) quaternion".format(quat_i, quat_j, quat_k, quat_real))

    # Get heading value
    quat_i, quat_j, quat_k, quat_real = sensor.get_rotation()
    heading = sensor.get_heading(quat_real, quat_i, quat_j, quat_k)
    print("Heading: {:.2f} degrees".format(heading))

    # Check for collision
    accel_mag = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
    if sensor.collision_detect(accel_mag):
        # Do something to handle the collision
        pass

    time.sleep(1)
    print("\n")
