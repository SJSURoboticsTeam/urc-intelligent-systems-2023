from gps_compass.actual_gps_compass import ActualGPSCompass
import time
gps = ActualGPSCompass()
try:
    while 1:
        print(gps.gps, gps.angle)
        time.sleep(1)
except KeyboardInterrupt:
    gps.disconnect()
