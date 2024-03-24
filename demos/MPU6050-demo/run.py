import sys
sys.path.append( '../../')
from proj_modules.MPU6050 import MPU6050

mpu = MPU6050()
mpu.read_data()