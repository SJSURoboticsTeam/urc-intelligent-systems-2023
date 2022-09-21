import sys
sys.path.append( '../../modules/GPS' )
from GPS import gpsRead

if __name__ == '__main__':
    data = gpsRead("/dev/ttyACM0",9600)
    while True:
        GPS_Coordinates = data.get_position()
        print(GPS_Coordinates)