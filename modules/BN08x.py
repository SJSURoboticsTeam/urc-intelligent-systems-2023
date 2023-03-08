import board
import busio
from math import atan2, sqrt, pi
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_STEP_COUNTER,
    BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C
import math

class BN08x:
    def __init__(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = BNO08X_I2C(i2c)
        # Enable required features
        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        self.bno.enable_feature(BNO_REPORT_STEP_COUNTER)
        self.bno.enable_feature(BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)


    def get_acceleration(self):
        """Get acceleration values (x, y, z) in m/s^2."""
        accel_x, accel_y, accel_z = self.bno.acceleration  # pylint:disable=no-member
        return (accel_x, accel_y, accel_z)


    def get_gyro(self):
        """Get gyro values (x, y, z) in degrees/sec."""
        gyro_x, gyro_y, gyro_z = self.bno.gyro  # pylint:disable=no-member
        return (gyro_x, gyro_y, gyro_z)
    

    def get_mag(self):
        """Get mag values (x, y, z) in microteslas."""
        mag_x, mag_y, mag_z = self.bno.magnetic
        return (mag_x, mag_y, mag_z)
    

    def get_rotation(self):
        """Get rotation values (i, j, k, real) in quaternion."""
        quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
        return (quat_i, quat_j, quat_k, quat_real)


    def get_heading(self, quat_i, quat_j, quat_k, quat_real):
        """Get heading in degrees from rotation vector."""
        norm = sqrt(quat_i * quat_i + quat_j * quat_j + quat_k * quat_k + quat_real * quat_real)
        quat_i = quat_i / norm
        quat_j = quat_j / norm
        quat_k = quat_k / norm
        quat_real = quat_real / norm

        ysqr = quat_k * quat_k

        t3 = +2.0 * (quat_i * quat_real + quat_j * quat_k)
        t4 = +1.0 - 2.0 * (ysqr + quat_real * quat_real)
        yaw_raw = atan2(t3, t4)
        yaw = yaw_raw * 180.0 / pi
        if yaw > 0:
            yaw = 360 - yaw
        else:
            yaw = abs(yaw)
        return yaw  # heading in 360 clockwise
    

    def collision_detect(self, accel_mag):
        """Detect collision."""
        if accel_mag > 8.0:
            print("Collision detected!")
            return True
        else:
            return False