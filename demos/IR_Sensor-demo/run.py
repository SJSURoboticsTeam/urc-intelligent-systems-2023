import sys
sys.path.append( '../../')
import time
from modules.IR_Sensor import IR_Sensor

sensor = 12
rover_detection = IR_Sensor(sensor)

try:
    while True:
            if rover_detection.DetectObject():
                print("Object Not Detected")
                time.sleep(0.5)
            else:
                print("Object Detected")
                time.sleep(0.5)
except KeyboardInterrupt:
    print("Exiting Program")