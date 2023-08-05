import random
from math import atan2, sqrt, pi
import time

class MockBN08x:
    def __init__(self):
        self.accel_range = (-10.0, 10.0)
        self.gyro_range = (-180.0, 180.0)
        self.mag_range = (-50.0, 50.0)
        self.quat_range = (-1.0, 1.0)

    def get_random_value(self, value_range):
        return random.uniform(value_range[0], value_range[1])

    def get_acceleration(self):
        """Get random acceleration values (x, y, z) in m/s^2."""
        return (self.get_random_value(self.accel_range),
                self.get_random_value(self.accel_range),
                self.get_random_value(self.accel_range))

    def get_gyro(self):
        """Get random gyro values (x, y, z) in degrees/sec."""
        return (self.get_random_value(self.gyro_range),
                self.get_random_value(self.gyro_range),
                self.get_random_value(self.gyro_range))

    def get_mag(self):
        """Get random mag values (x, y, z) in microteslas."""
        return (self.get_random_value(self.mag_range),
                self.get_random_value(self.mag_range),
                self.get_random_value(self.mag_range))

    def get_rotation(self):
        """Get random rotation values (i, j, k, real) in quaternion."""
        return (self.get_random_value(self.quat_range),
                self.get_random_value(self.quat_range),
                self.get_random_value(self.quat_range),
                self.get_random_value(self.quat_range))

    def get_geomagnetic_rotation(self):
        """Get random geomagnetic rotation values (i, j, k, real) in quaternion."""
        return (self.get_random_value(self.quat_range),
                self.get_random_value(self.quat_range),
                self.get_random_value(self.quat_range),
                self.get_random_value(self.quat_range))

    def get_heading(self, quat_i, quat_j, quat_k, quat_real):
        """Get random heading in degrees from rotation vector."""
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

    def collision_detect(self, threshold=8.0, duration=0.1):
        """Detect collision."""
        accel_list = []
        start_time = time.monotonic()

        # Collect acceleration data for duration
        while time.monotonic() - start_time < duration:
            accel = self.get_acceleration()
            accel_mag = sqrt(sum(a ** 2 for a in accel))
            accel_list.append(accel_mag)

        # Check if maximum acceleration exceeds threshold
        max_accel = max(accel_list)
        if max_accel > threshold:
            print("Collision detected! Max acceleration: {:.2f}".format(max_accel))
            return True
        else:
            return False

