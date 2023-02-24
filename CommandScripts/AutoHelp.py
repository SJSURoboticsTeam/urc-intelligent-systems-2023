import math
from modules.LSM303 import Compass
import json

class AutoHelp:
    def get_distance(self, current_GPS, target_GPS):
            R_KM = 6373.0
            R_MI = 3958.8
            try:
                current_lat = math.radians(current_GPS[1])
                current_lon = math.radians(current_GPS[0])
                target_lat = math.radians(target_GPS[1])
                target_lon = math.radians(target_GPS[0])

                dis_lon = target_lon - current_lon
                dis_lat = target_lat - current_lat

                form1 = math.sin(dis_lat / 2)**2 + math.cos(current_lat) * math.cos(target_lat) * math.sin(dis_lon / 2)**2
                form2 = 2 * math.atan2(math.sqrt(form1), math.sqrt(1 - form1))

                distanceKM = R_KM * form2
                distanceMi = R_MI * form2
                return [distanceKM, distanceMi]
            except:
                print("No GPS Data")


    def get_spin_angle(self, current_GPS, target_GPS):
            x = target_GPS[1] - current_GPS[1]
            y = target_GPS[0] - current_GPS[0]
            quad = 0
            angle = 0

            compass = Compass()
            heading = compass.get_heading()

            if x < 0:
                if y < 0:
                    quad = 3
                else:
                    quad = 2
            else:
                if y < 0:
                    quad = 4
                else:
                    quad = 1
                    
            angle += 360-heading

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

            if angle < (360-angle):
                return angle
            else:
                return -abs(360-angle)


    def jsonify_commands(self, commands):
        json_command = {"HB":commands[0],"IO":commands[1],"WO":commands[2],"DM":f"{commands[3]}","CMD":[commands[4],commands[5]]}
        json_command = json.dumps(json_command)
        json_command = json_command.replace(" ", "")
        return json_command


    def get_bearing(self, current_GPS, target_GPS):
        current_latitude = math.radians(current_GPS[1])
        current_longitude = math.radians(current_GPS[0])
        target_latitude = math.radians(target_GPS[1])
        target_longitude = math.radians(target_GPS[0])

        deltalog= target_longitude-current_longitude;

        x=math.cos(target_latitude)*math.sin(deltalog);
        y=(math.cos(current_latitude)*math.sin(target_latitude))-(math.sin(current_latitude)*math.cos(target_latitude)*math.cos(deltalog));

        bearing=(math.atan2(x,y))*(180/3.14);
        return bearing
    