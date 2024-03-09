from time import sleep
import board
import adafruit_lsm303dlh_mag
import math


class Compass:
    # calibration results from calibrate.py
    hard_iron_bias_x = 8.33426572
    hard_iron_bias_y = 38.4854981
    hard_iron_bias_z = 0.51834922

    soft_iron_bias_xx = 20.097355661177684
    soft_iron_bias_xy = 0.20530722218264985
    soft_iron_bias_xz = 0.544377527437325

    soft_iron_bias_yx = 0.20530722218265313
    soft_iron_bias_yy = 20.0365536823096
    soft_iron_bias_yz = 1.1208798385500047

    soft_iron_bias_zx = 0.5443775274373277
    soft_iron_bias_zy = 1.12087983855
    soft_iron_bias_zz = 19.24727698802664

    def __init__(self):
        self.i2c = board.I2C()

        self.mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(self.i2c)
        self.mag.mag_rate = adafruit_lsm303dlh_mag.MAGRATE_75

    def get_heading(self):
        # x, y, z = self.mag.magnetic
        x_off = self.mag.magnetic[0] - self.hard_iron_bias_x
        y_off = self.mag.magnetic[1] - self.hard_iron_bias_y
        z_off = self.mag.magnetic[2] - self.hard_iron_bias_z
        x = (
            x_off * self.soft_iron_bias_xx
            + y_off * self.soft_iron_bias_yx
            + z_off * self.soft_iron_bias_zx
        )
        y = (
            x_off * self.soft_iron_bias_xy
            + y_off * self.soft_iron_bias_yy
            + z_off * self.soft_iron_bias_zy
        )
        # z = (
        #     x_off * self.soft_iron_bias_xz
        #     + y_off * self.soft_iron_bias_yz
        #     + z_off * self.soft_iron_bias_zz
        # )

        heading = math.atan2(y, x) * (180 / math.pi)

        # use y as the anchor, not x
        # so that we get 0* as North
        # this is done by just subtracting 90*
        heading -= 90

        if heading < 0:
            heading += 360
        return round(heading, 3)

    def get_raw(self):
        return self.mag.magnetic
