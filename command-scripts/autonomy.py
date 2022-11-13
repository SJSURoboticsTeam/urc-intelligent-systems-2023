import math
import sys

sys.path.append( '/home/pi/Repos/urc-intelligent_systems-2022/modules/LSM303')
from LSM303 import Compass


class Autonomy():

    def get_distance(self, lon1 ,lat1, lon2, lat2):

        R_KM = 6373.0
        R_MI = 3958.8
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)

        dis_lon = lon2 - lon1
        dis_lat = lat2 - lat1

        form1 = math.sin(dis_lat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dis_lon / 2)**2
        form2 = 2 * math.atan2(math.sqrt(form1), math.sqrt(1 - form1))

        distanceKM = R_KM * form2
        distanceMi = R_MI * form2
        return [distanceKM, distanceMi]

    def get_spin_angle(self, lon1, lat1, lon2, lat2):
        x = lat2 - lat1
        y = lon2 - lon1
        quad = 0
        angle = 0

        compass = Compass()
        heading = compass.get_heading()
        print(heading)

        if x < 0:
            if y < 0:
                print("3")
                quad = 3
            else:
                print("2")
                quad = 2
        else:
            if y < 0:
                print("4")
                quad = 4
            else:
                print("1")
                quad = 1
                
        angle += 360-heading
        print(angle%360)

        if quad == 1:
            angle += 90-angle
            angle -= math.degrees(math.atan(abs(y)/abs(x)))
        elif quad == 2:
            angle += 360-angle
            angle -= math.degrees(math.atan(abs(y)/abs(x)))
        elif quad == 3:
            angle += 270-angle
            angle -= 90 - math.degrees(math.atan(abs(y)/abs(x)))
        elif quad == 4:
            angle += 180-angle
            angle -= 90 - math.degrees(math.atan(abs(y)/abs(x)))

        return angle

auto = Autonomy()
print(auto.get_angle(24, 36, 12, 52))
