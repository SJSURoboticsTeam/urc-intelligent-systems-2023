import sys
import time
sys.path.append( '../../')
from proj_modules.LSM303 import Compass

if __name__ == '__main__':
    try:
        compass = Compass()
        print("Compass initialized, try to keep it away from other electronics that might interfere with the magnetic field")
    except:
        print("Make sure your LSM303 is plugged in!")
        exit(1)

    while True:
        # x,y,z = compass.get_raw()
        # print(f"{x},{y},{z}")
        heading = compass.get_heading()
        print(str(heading) + chr(176))
        time.sleep(1/10)
