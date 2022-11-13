import math
import json
from modules.LSM303 import Compass

def get_distance(lon1 ,lat1, lon2, lat2):

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

def jsonify_commands(commands):
    json_command = {"HB":commands[0],"IO":commands[1],"WO":commands[2],"DM":f"{commands[3]}","CMD":[commands[4],commands[5]]}
    json_command = json.dumps(json_command)
    return json_command


def get_bearing(lon1, lat1, lon2, lat2):
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    deltalog= lon2-lon1;

    x=math.cos(lat2)*math.sin(deltalog);
    y=(math.cos(lat1)*math.sin(lat2))-(math.sin(lat1)*math.cos(lat2)*math.cos(deltalog));

    bearing=(math.atan2(x,y))*(180/3.14);
    return bearing


def forward_rover():
    commands = [0,0,0,'D',50,0]
    jsonify_commands(commands)

def steer_left():
    commands = [0,0,0,'D',0,-12]
    jsonify_commands(commands)

def steer_right():
    commands = [0,0,0,'D',0,12]
    jsonify_commands(commands)

def stop_rover():
    commands = [0,0,0,'D',0,0]
    jsonify_commands(commands)

def get_steering(lon1, lat1, lon2, lat2):
    final_angle = Compass.get_heading()/get_bearing(lon1, lat1, lon2, lat2)

    if(final_angle >= 0 and final_angle <= 1):
        forward_rover()
        

    elif(final_angle > 1 and final_angle <= 8):
        steer_left()
        

    elif(final_angle <= 13 and final_angle >= 8):
        steer_right()
        

    elif(lon2==lon1 and lat1==lat2):
        stop_rover()