import serial
import requests
import math

class gpsRead:
    def __init__(self, port, baudrate):
        self.gps_port = serial.Serial(port, baudrate)

    def send_request(self, LonLat, url):
        requests.post(url, data={"longitude": LonLat["longitude"], "latitude": LonLat["latitude"]})

    def get_position(self, url):
        LonLat = {"longitude":0,
                  "latitude":0}
        try:
            s = (self.gps_port.read(500)).decode('utf-8')
            data = s.splitlines()
            for i in range(len(data)):
                d = data[i].split(',')
                if d[0] == "$GPGGA" and len(d) == 15:
                    if d[2] == '' or d[4] == '':
                        return ["None", "None"]
                    else:
                        lat = float(d[2]) / 100
                        long = float(d[4]) / 100
                        lat = math.modf(lat)
                        long = math.modf(long)
                        lat = lat[1]+(lat[0]*100)/60
                        long = long[1]+(long[0]*100)/60
                        LonLat["longitude"] = -long
                        LonLat["latitude"]= lat
                        self.send_request(LonLat, url)
                        return [long, -lat]
        except:
            return ['error', 'error']
