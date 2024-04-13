"""This script is responsible for taking in the path from pathfinder 
and determining a command to send to the rover"""

import sys
import re

root = next(re.finditer(".*unified_frameworks", __file__)).group()
sys.path.append(root) if root not in sys.path else None
from math import pi
from unified_utils import Service
import sys, os

try:
    from proj_modules.WiFi import WiFi, make_drive_command, Modes
except:
    sys.path.append(os.path.realpath(__file__ + os.sep + ".." + os.sep + ".."))
    from proj_modules.WiFi import WiFi, make_drive_command, Modes
from unified_frameworks.sensor_array.gps_compass import gps_compass
import pathfinder_visualizer
import time
from straight_shot import StraightShot
import rapid_random_tree
from rapid_random_tree import RRT_Navigator
import worldview
import pathfinder as _pathfinder
import importlib
from typing import Tuple

importlib.reload(worldview)
importlib.reload(_pathfinder)
import straight_shot

importlib.reload(straight_shot)
# importlib.reload(gps_compass)
# importlib.reload(rapid_random_tree)
# pathfinder = RRT_Navigator(worldview)
# pathfinder = StraightShot(worldview)


# parameters that control how the pathfinder service works
config = {
    "command_frequency": 10,  # Hz
    "verbose_service_events": True,
    "send_commands_to_rover": False,
    "verbose_rover_commands": True,
    "distance_threshold": 3,  # how close, in meters, till "at a destination"
}

# globals used for captain service
pathfinder = _pathfinder.get_pathfinder()
_command_printer = print
rover = WiFi("http://192.168.0.211:5000")
cur_rad = 0
rad_lag = 0.9
_get_target_speed = lambda: 0
_gps_coordinates = [  # lat, long pairs
    (-121.8818545, 37.3370768),
    (-121.8818535, 37.3370083),
    (-121.8817744, 37.3369864),
]
_cur_gps_coordinate: Tuple[int, int] = None


def captain_act(get_target_speed):
    print("Captain acting")
    path = pathfinder.get_path()
    if len(path) < 2:
        command = make_drive_command(speed_percent=0)
        rover.send_command(command) if config["send_commands_to_rover"] else None
        print(command) if config["verbose_rover_commands"] else None
        radians = 0
        return
    else:
        nxt = path[1]
        radians = (nxt[0] % (2 * pi)) - pi / 2
    radians *= -1
    global cur_rad
    radians = (1 - rad_lag) * radians + rad_lag * cur_rad
    cur_rad = radians
    degrees = int(radians / (2 * pi) * 360)
    mode = Modes.DRIVE if abs(degrees) < 30 else Modes.SPIN
    speed = get_target_speed()
    if mode == Modes.SPIN:
        speed *= -1 if degrees < 0 else 1
    # print(mode, speed)
    if False:  # If flipped
        degrees *= -1
        if mode == Modes.DRIVE:
            speed *= -1
    command = make_drive_command(mode, speed_percent=speed, angle_degrees=degrees)
    rover.send_command(command) if config["send_commands_to_rover"] else None

    print(command) if config["verbose_rover_commands"] else None
    # rover.send_command(command)


def captain_stop():
    command = make_drive_command(speed_percent=0)
    rover.send_command(command) if config["send_commands_to_rover"] else None
    with _command_printer:
        print(command)


def set_goal_coordinates(coordinate: Tuple[int, int]):
    """Set the latitude, longitude for the goal"""
    global _cur_gps_coordinate
    _cur_gps_coordinate = coordinate


def at_coordinate_function():
    """
    Function to run when at a GPS coordinate. Currently a stub, but should eventually
    be a function that navigates to the aruco tag at a GPS coordinate.
    """
    print("At GPS coordinate")
    captain_act(lambda: 0)
    time.sleep(5)


def run_captain(is_captain_running):
    pathfinder.start_pathfinder_service()
    coordinate_iterable = iter(_gps_coordinates)
    set_goal_coordinates(next(coordinate_iterable))
    while is_captain_running() and _cur_gps_coordinate is not None:
        print("Running")

        pathfinder.set_gps_goal(*_cur_gps_coordinate)
        captain_act(_get_target_speed)
        time.sleep(1 / config["command_frequency"])

        # go to next point if "at goal"
        # if r < distance threshold
        if pathfinder.distance_to_target() < config["distance_threshold"]:
            at_coordinate_function()
            set_goal_coordinates(next(coordinate_iterable, None))

    captain_stop()
    pathfinder.stop_pathfinder_service()
    gps_compass.disconnect()


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
    return pathfinder_visualizer.show_visual(_pathfinder.get_pathfinder)


if __name__ == "__main__":
    start_captain_service(lambda: 10, print)
    pathfinder_visualizer.show_visual(lambda: pathfinder)
    stop_captain_service()
