import sys
import math
import json
import requests
import serial.tools.list_ports as port_list
sys.path.append( '/home/pi/Repos/urc-intelligent_systems-2022/modules/LSM303')
sys.path.append( '/home/pi/Repos/urc-intelligent_systems-2022/modules/Serial')
sys.path.append( '/home/pi/Repos/urc-intelligent_systems-2022/modules/GPS')
from LSM303 import Compass
from GPS import gpsRead
from Serial import SerialSystem

class Autonomy:
    def __init__(self, port, baudrate, url):
        self.port = port
        self.baudrate = baudrate
        self.url = url
        self.max_speed = 20
        self.max_steering = 12
        self.commands = [0,0,0,'D',0,0]
        self.current_GPS = [0,0]
        self.desired_GPS = [-121.881073,37.335186]
        self.serial = SerialSystem(self.port, self.baudrate)
        self.serial.connect()
        self.connect_GPS()


    def connect_GPS(self):
        while True:
            try:
                self.GPS_data = gpsRead("/dev/ttyACM0",9600)
            except:
                print("Make sure your GPS is plugged in and you are using the correct port!")
                continue
            break

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
        return json_command


    def get_bearing(self, lon1, lat1, lon2, lat2):
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)

        deltalog= lon2-lon1;

        x=math.cos(lat2)*math.sin(deltalog);
        y=(math.cos(lat1)*math.sin(lat2))-(math.sin(lat1)*math.cos(lat2)*math.cos(deltalog));

        bearing=(math.atan2(x,y))*(180/3.14);
        return bearing


    def forward_rover(self, commands):
        self.commands[4] = self.max_speed
        self.jsonify_commands(commands)

    def steer_left(self, commands):
        self.commands[5] = -self.max_steering
        self.jsonify_commands(commands)

    def steer_right(self, commands):
        self.commands[5] = self.max_steering
        self.jsonify_commands(commands)

    def stop_rover(self, commands):
        self.commands = [0,0,0,'D',0,0]
        self.jsonify_commands(commands)


    def get_steering(self, lon1, lat1, lon2, lat2):
        
        final_angle = Compass.get_heading()/self.get_bearing(lon1, lat1, lon2, lat2)

        if(final_angle >= 0 and final_angle <= 1):
            self.forward_rover(self.commands)
            
        elif(final_angle > 1 and final_angle <= 8):
            self.steer_left(self.commands)
            

        elif(final_angle <= 13 and final_angle >= 8):
            self.steer_right(self.commands)
            

        elif(lon2==lon1 and lat1==lat2):
            self.stop_rover(self.commands)

    def get_rover_status(self):
        bearing = self.get_bearing(self.current_GPS[0], self.current_GPS[1], self.desired_GPS[0], self.desired_GPS[1])
        distance = self.get_distance(self.current_GPS[0], self.current_GPS[1], self.desired_GPS[0], self.desired_GPS[1])
        GPS = self.current_GPS
        json_command = {"Bearing":bearing,"Distance":distance,"GPS":[GPS[0],GPS[1]]}
        json_command = json.dumps(json_command)
        requests.post(self.url, data=None, json=json_command)

    def start_mission(self):

        try:
            serial = SerialSystem(self.port, self.baudrate)
            print("Using port: " + self.port)
        except:
            ports = list(port_list.comports())
            print('====> Designated Port not found. Using Port:', ports[0].device)
            self.port = ports[0].device
            serial = SerialSystem(self.port, self.baudrate)

        homing_end = "Starting control loop..."
        while True:
            response = serial.read_serial()
            if homing_end in response:
                while True:
                    self.current_GPS = self.GPS_data.get_position(f"{self.url}/gps")
                    command = self.get_steering(self.current_GPS[0], self.current_GPS[1], self.desired_GPS[0], self.desired_GPS[1])
                    response += serial.read_serial()
                    serial.write_serial(command.text)
                    self.get_rover_status()
                    
