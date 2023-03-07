import time
import board
import adafruit_lsm303dlh_mag
import adafruit_lsm303agr
import adafruit_l3gd20
import math

class BN08x:
    def __init__(self):
        self.i2c = board.I2C()
        self.mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(self.i2c)
        self.accel = adafruit_lsm303agr.LSM303AGR_Accelerometer(self.i2c)
        self.gyro = adafruit_l3gd20.L3GD20_I2C(self.i2c)

        # Set accelerometer range to +/- 4g
        self.accel.range = adafruit_lsm303agr.Range.RANGE_4G

        # Set gyro range to +/- 500 degrees/sec
        self.gyro.range = adafruit_l3gd20.Range.RANGE_500DPS

        # Enable magnetometer continuous measurement
        self.mag.measurement_mode = adafruit_lsm303dlh_mag.MeasurementMode.CONTINUOUS

        # Calibrate accelerometer, gyro, and magnetometer
        self.calibrate()

    def twos_complement(self, val, bits):
        """Convert an integer to two's complement."""
        if val & (1 << (bits - 1)):
            val -= 1 << bits
        return val

    def calibrate(self):
        """Calibrate accelerometer, gyro, and magnetometer."""
        print("Calibrating...")

        # Calculate accelerometer offsets
        accel_x_sum = 0
        accel_y_sum = 0
        accel_z_sum = 0
        for i in range(1000):
            accel_x_raw, accel_y_raw, accel_z_raw = self.accel.acceleration
            accel_x_sum += accel_x_raw
            accel_y_sum += accel_y_raw
            accel_z_sum += accel_z_raw
            time.sleep(0.01)
        self.accel_x_offset = -accel_x_sum / 1000
        self.accel_y_offset = -accel_y_sum / 1000
        self.accel_z_offset = (8192 - accel_z_sum) / 1000

        # Calculate gyro offsets
        gyro_x_sum = 0
        gyro_y_sum = 0
        gyro_z_sum = 0
        for i in range(1000):
            gyro_x_raw, gyro_y_raw, gyro_z_raw = self.gyro.gyro
            gyro_x_sum += gyro_x_raw
            gyro_y_sum += gyro_y_raw
            gyro_z_sum += gyro_z_raw
            time.sleep(0.01)
        self.gyro_x_offset = -gyro_x_sum / 1000
        self.gyro_y_offset = -gyro_y_sum / 1000
        self.gyro_z_offset = -gyro_z_sum / 1000

    def get_acceleration(self):
        """Get acceleration values (x, y, z) in m/s^2."""
        accel_x_raw, accel_y_raw, accel_z_raw = self.accel.acceleration
        accel_x = (accel_x_raw + self.accel_x_offset) * self.accel.scale / 1000
        accel_y = (accel_y_raw + self.accel_y_offset) * self.accel.scale / 1000
        accel_z = (accel_z_raw + self.accel_z_offset) * self.accel.scale / 1000
        return (accel_x, accel_y, accel_z)

    def get_gyro(self):
        """Get gyro values (x, y, z) in degrees/sec."""
        gyro_x_raw, gyro_y_raw, gyro_z_raw = self.gyro.gyro
        gyro_x = (gyro_x_raw + self.gyro_x_offset) * self.gyro.scale
        gyro_y = (gyro_y_raw + self.gyro_y_offset) * self.gyro.scale
        gyro_z = (gyro_z_raw + self.gyro_z_offset) * self.gyro.scale
        return (gyro_x, gyro_y, gyro_z)

    def get_heading(self):
        """Get heading in degrees from magnetometer."""
        mag_x_raw, mag_y_raw, mag_z_raw = self.mag.magnetic
        mag_x = mag_x_raw * 0.92
        mag_y = mag_y_raw * 0.92
        mag_z = mag_z_raw * 0.92
        heading = math.atan2(mag_y, mag_x) * (180 / math.pi)
        if heading < 0:
            heading += 360
        return heading
    
    def collision_detect(self, accel_mag):
        """Detect collision."""
        if accel_mag > 8.0:
            print("Collision detected!")
            return True
        else:
            return False
    
    def update(self):
        """Update state variables."""
        current_time = time.monotonic()
        delta_time = current_time - self.last_time

        # Calculate acceleration
        accel_x_raw, accel_y_raw, accel_z_raw = self.accel.acceleration
        accel_x = (accel_x_raw + self.accel_x_offset) * self.accel.scale / 1000
        accel_y = (accel_y_raw + self.accel_y_offset) * self.accel.scale / 1000
        accel_z = (accel_z_raw + self.accel_z_offset) * self.accel.scale / 1000
        acceleration = (accel_x, accel_y, accel_z)

        # Calculate velocity
        velocity_x = self.last_velocity[0] + (self.last_acceleration[0] + acceleration[0]) / 2 * delta_time
        velocity_y = self.last_velocity[1] + (self.last_acceleration[1] + acceleration[1]) / 2 * delta_time
        velocity_z = self.last_velocity[2] + (self.last_acceleration[2] + acceleration[2]) / 2 * delta_time
        velocity = (velocity_x, velocity_y, velocity_z)

        # Calculate position
        position_x = self.last_position[0] + (self.last_velocity[0] + velocity[0]) / 2 * delta_time
        position_y = self.last_position[1] + (self.last_velocity[1] + velocity[1]) / 2 * delta_time
        position_z = self.last_position[2] + (self.last_velocity[2] + velocity[2]) / 2 * delta_time
        position = (position_x, position_y, position_z)

        # Calculate orientation using gyro
        gyro_x_raw, gyro_y_raw, gyro_z_raw = self.gyro.gyro
        gyro_x = (gyro_x_raw + self.gyro_x_offset) * self.gyro.scale
        gyro_y = (gyro_y_raw + self.gyro_y_offset) * self.gyro.scale
        gyro_z = (gyro_z_raw + self.gyro_z_offset) * self.gyro.scale
        self.orientation = (self.orientation[0] + gyro_x * delta_time,
                             self.orientation[1] + gyro_y * delta_time,
                             self.orientation[2] + gyro_z * delta_time)

        # Detect collision using accelerometer
        acceleration_magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
        self.collision_detect(acceleration_magnitude)

        # Update state variables
        self.last_time = current_time
        self.last_position = position
        self.last_velocity = velocity
        self.last_acceleration = acceleration


