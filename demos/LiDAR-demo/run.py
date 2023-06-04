import sys
sys.path.append('../../')
from Vision.modules.LiDARModule import LiDARModule
import time

PORT_NAME = '/dev/ttyUSB2'

lidar_plot = LiDARModule(PORT_NAME)
start_time = time.time()

try:
    for output in lidar_plot.run():
        print(output)
        if time.time() - start_time > 600:  
            break
except KeyboardInterrupt:
    print('Stopping.')
finally:
    if lidar_plot.is_connected:
        lidar_plot.stop()



