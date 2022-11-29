import sys
sys.path.append( '../../')
from modules.MPU6050 import MPU6050

mpu = MPU6050()
mpu.read_data()