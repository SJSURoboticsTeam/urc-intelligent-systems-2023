from time import sleep
import board
import busio
import adafruit_lsm303dlh_mag
import math

class Compass:
    def __init__(self):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(self.i2c)

    def get_heading(self):
        x = round(self.mag.magnetic[0], 3)
        y = round(self.mag.magnetic[1], 3)
        heading = math.atan2(y,x)*(180/math.pi)
        if heading < 0:
            heading += 360
        return round(heading, 3)