import sys
import re
root = (next(re.finditer(".*sensor_array", __file__)).group())
sys.path.append(root) if root not in sys.path else None
from gps_compass.actual_gps_compass import ActualGPSCompass
import time

if __name__=='__main__':
    gps = ActualGPSCompass()
    try:
        while 1:
            print(gps.get_cur_gps(), gps.get_cur_angle())
            time.sleep(1)
    except KeyboardInterrupt:
        gps.disconnect()
