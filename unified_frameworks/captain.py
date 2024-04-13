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
import pathfinder_visualizer
import time
import unified_frameworks.pathfinders.pathfinder as _pathfinder
import importlib
from typing import Tuple, List, Callable

importlib.reload(_pathfinder)


# parameters that control how the pathfinder service works
config = {
    "command_frequency": 10,  # Hz
    "verbose_service_events": True,
    "send_commands_to_rover": False,
    "verbose_rover_commands": True,
    "distance_threshold": 3,  # how close, in meters, till "at a destination"
}


class Captain(Service):
    def __init__(
        self,
        gps_targets: List[Tuple[float, float]],
        get_target_speed: Callable[[], float],
    ) -> None:
        """
        Create the Captain that is in charge of navigating the rover.

        Arguments:
            - gps_targets: List[Tuple[float, float]] - the GPS positions to navigate to; these should
            be pairs of (latitude, longitude)
            - get_target_speed: Callable[[], float] - a function that determines the speed at which the
            rover should move.
        """
        super().__init__(lambda is_alive: self.run_captain(is_alive), "Captain Service")

        # globals used for captain service
        self.pathfinder = _pathfinder.Pathfinder()
        self.rover = WiFi("http://192.168.0.211:5000")
        self.cur_rad = 0
        self.rad_lag = 0.9
        self._get_target_speed = lambda: 0
        self._gps_coordinates = gps_targets
        self._cur_gps_coordinate: Tuple[int, int] = None
        self._get_target_speed = get_target_speed
        self.finished = False

    def captain_act(self):
        print("Captain acting")
        path = self.pathfinder.get_path()
        if len(path) < 2:
            command = make_drive_command(speed_percent=0)
            (
                self.rover.send_command(command)
                if config["send_commands_to_rover"]
                else None
            )
            print(command) if config["verbose_rover_commands"] else None
            radians = 0
            return
        else:
            nxt = path[1]
            radians = (nxt[0] % (2 * pi)) - pi / 2
        radians *= -1
        radians = (1 - self.rad_lag) * radians + self.rad_lag * self.cur_rad
        self.cur_rad = radians
        degrees = int(radians / (2 * pi) * 360)
        mode = Modes.DRIVE if abs(degrees) < 30 else Modes.SPIN
        speed = self._get_target_speed()
        if mode == Modes.SPIN:
            speed *= -1 if degrees < 0 else 1
        command = make_drive_command(mode, speed_percent=speed, angle_degrees=degrees)
        self.rover.send_command(command) if config["send_commands_to_rover"] else None

        print(command) if config["verbose_rover_commands"] else None

    def captain_stop(self):
        command = make_drive_command(speed_percent=0)
        self.rover.send_command(command) if config["send_commands_to_rover"] else None
        print(command) if config["verbose_rover_commands"] else None

    def at_coordinate_function(self):
        """
        Function to run when at a GPS coordinate. Currently a stub, but should eventually
        be a function that navigates to the aruco tag at a GPS coordinate.
        """
        print("At GPS coordinate")
        self.captain_act(lambda: 0)
        time.sleep(5)

    def run_captain(self, is_captain_running: Callable[[], bool]):
        self.pathfinder.start_pathfinder_service()
        coordinate_iterable = iter(self._gps_coordinates)
        self._cur_gps_coordinate = next(coordinate_iterable, None)
        while is_captain_running() and self._cur_gps_coordinate is not None:
            print("Running")

            self.pathfinder.set_gps_goal(*self._cur_gps_coordinate)
            self.captain_act()
            time.sleep(1 / config["command_frequency"])

            # go to next point if "at goal"
            # if r < distance threshold
            if self.pathfinder.distance_to_target() < config["distance_threshold"]:
                self.at_coordinate_function()
                self._cur_gps_coordinate = next(coordinate_iterable, None)

        self.captain_stop()
        self.pathfinder.stop_pathfinder_service()
        self.finished = True


if __name__ == "__main__":
    captain = Captain(
        [
            (-121.8818545, 37.3370768),
            (-121.8818535, 37.3370083),
            (-121.8817744, 37.3369864),
        ],
        get_target_speed=lambda: 10,
    )
    captain.start_service()
    pathfinder_visualizer.show_visual(lambda: captain.pathfinder)
    captain.stop_service()
