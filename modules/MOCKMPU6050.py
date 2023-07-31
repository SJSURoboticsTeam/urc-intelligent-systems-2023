import random
from time import sleep

class MockMPU6050:
    def __init__(self):
        self.Device_Address = 0x68

    def MPU_Init(self):
        # Mock MPU_Init, no actual initialization needed for the mock
        pass

    def read_raw_data(self, addr):
        # Generate random raw accelerometer and gyroscope values as if they were read from the sensor
        return random.randint(-32768, 32767)

    def read_data(self):
        # Pretend to init MPU
        self.MPU_Init()

        while True:
            # Generate random accelerometer and gyroscope values for simulation
            acc_x = self.read_raw_data(0x3B)
            acc_y = self.read_raw_data(0x3D)
            acc_z = self.read_raw_data(0x3F)
            gyro_x = self.read_raw_data(0x43)
            gyro_y = self.read_raw_data(0x45)
            gyro_z = self.read_raw_data(0x47)

            # Convert raw values to scaled values
            Ax = acc_x / 16384.0
            Ay = acc_y / 16384.0
            Az = acc_z / 16384.0

            Gx = gyro_x / 131.0
            Gy = gyro_y / 131.0
            Gz = gyro_z / 131.0

            # Print the simulated sensor values
            print("Gx=%.2f" % Gx, u'\u00b0' + "/s", "\tGy=%.2f" % Gy, u'\u00b0' + "/s", "\tGz=%.2f" % Gz, u'\u00b0' + "/s",
                  "\tAx=%.2f g" % Ax, "\tAy=%.2f g" % Ay, "\tAz=%.2f g" % Az)
            sleep(1)

