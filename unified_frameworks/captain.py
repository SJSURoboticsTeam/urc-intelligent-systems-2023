"""This script is responsible for taking in the path from pathfinder 
and determining a command to send to the rover"""

from math import pi
from unified_utils import Service
import sys, os
try:
    from modules.WiFi import WiFi, make_drive_command, Modes
except:
    sys.path.append(os.path.realpath(__file__+os.sep+".."+os.sep+".."))
    from modules.WiFi import WiFi, make_drive_command, Modes
import pathfinder_visualizer
import time
from straight_shot import StraightShot

pathfinder = StraightShot()

config = {
    "command_frequency": 10, #Hz
    "verbose_service_events": True,
    "send_commands_to_rover": False,
}

_command_printer = print
rover = WiFi("http://192.168.0.211:5000")
prev_rad = 0
def captain_act(get_target_speed):
    path = pathfinder.get_path()
    if len(path) < 2:
        command = make_drive_command(speed_percent=0)
        rover.send_command(command) if config['send_commands_to_rover'] else None
        _command_printer(command)
        radians=0
        return
    else:
        nxt = path[1]
        radians = (nxt[0]%(2*pi))-pi/2
    radians*=-1
    global prev_rad
    radians = 0.9*radians + 0.1*prev_rad
    prev_rad = radians
    degrees = int(radians/(2*pi)*360)
    mode = Modes.DRIVE if abs(degrees) < 30 else Modes.SPIN
    speed = get_target_speed()
    if mode == Modes.SPIN:
        speed *= (-1 if degrees < 0 else 1)
    # print(mode, speed)
    if False: #If flipped
        degrees *= -1
        if mode == Modes.DRIVE:
            speed *= -1
    command = make_drive_command(mode, speed_percent=speed, angle_degrees=degrees)
    rover.send_command(command) if config['send_commands_to_rover'] else None
    _command_printer(command) 
    # rover.send_command(command)

def captain_stop():
    command = make_drive_command(speed_percent=0)
    rover.send_command(command) if config['send_commands_to_rover'] else None
    _command_printer(command)

_get_target_speed = lambda: 0
def run_captain(is_captain_running):
    pathfinder.start_pathfinder_service()
    while is_captain_running():
        captain_act(_get_target_speed)
        time.sleep(1/config['command_frequency'])
    captain_stop()
    pathfinder.stop_pathfinder_service()

_service = Service(run_captain, "Captain Service")
def start_captain_service(get_target_speed, command_printer):
    if config["verbose_service_events"]:
        print("Starting Captain Service")
    global _get_target_speed, _command_printer
    _get_target_speed = get_target_speed
    _command_printer = command_printer
    _service.start_service()
    # visual = pathfinder_visualizer.show_visual(pathfinder)
    # return visual
def stop_captain_service():
    if config["verbose_service_events"]:
        print("Stop Captain Service")
    _service.stop_service()
def show_captain_visual():
    return pathfinder_visualizer.show_visual(pathfinder)


if __name__=='__main__':
    start_captain_service(lambda: 10)
    pathfinder_visualizer.show_visual(pathfinder)
    stop_captain_service()



    