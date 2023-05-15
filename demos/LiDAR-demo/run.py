import sys
sys.path.append('../../')
from Vision.modules.LiDARModule import LiDARModule
import time

PORT_NAME = '/dev/ttyUSB0'

lidar_plot = LiDARModule(PORT_NAME)

while True:
    try:
        for output in lidar_plot.run():
            print(output)
    except KeyboardInterrupt:
        lidar_plot.stop()
        break
    except:
        continue