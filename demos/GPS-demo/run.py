import sys
sys.path.append( '../../modules/GPS' )
from GPS import gpsRead

if __name__ == '__main__':
    try:
        data = gpsRead("/dev/ttyACM0",9600)
        url = "http://192.168.50.243:5000/gps"
    except:
            print("Make sure your GPS is plugged in and you are using the correct port!")
            exit(1)
    GPS_Coordinates = data.get_position(url)
    print(GPS_Coordinates)
    exit(1)