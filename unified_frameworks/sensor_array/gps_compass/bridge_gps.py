from typing import Tuple
import time
import sys
import re

root = next(re.finditer(".*unified_frameworks", __file__)).group()
sys.path.append(root) if root not in sys.path else None
from bridge import rover_side, client_side
from bridge.exceptions import NoOpenBridgeException
from unified_utils import Service

try:
    from sensor_array.gps_compass.actual_gps_compass import ActualGPSCompass
except ImportError as e:
    print(e.msg)
    print("Assuming this means on client side.")
from sensor_array.gps_compass.gps_compass_class import _GPSCompass

import json

config = {
    "update_frequency": 2,  # Hz
}


class BridgeGPS(_GPSCompass):
    PATH = "/gps"

    def __init__(self) -> None:
        super().__init__()
        if rover_side.bridge_is_up():
            assert (
                not client_side.bridge_is_up()
            ), "Both Rover and Client side bridge is up! Whats going on here??"
            self.gps = ActualGPSCompass()
            self.gps.start_service()
            self.data = rover_side.data
            self.ON_ROVER_SIDE = True
            self.ON_CLIENT_SIDE = not self.ON_ROVER_SIDE
            return
        elif client_side.bridge_is_up():
            self.data = client_side.data
            self.ON_ROVER_SIDE = False
            self.ON_CLIENT_SIDE = not self.ON_ROVER_SIDE
            return
        raise NoOpenBridgeException("Neither Rover nor Client side of bridge is up!")

    def connect(self, max_attempts=3, wait_seconds=1, verbose_attempts=False) -> bool:
        if self.ON_CLIENT_SIDE:
            for attempt in range(max_attempts):
                if BridgeGPS.PATH in self.data:
                    return True
                time.sleep(wait_seconds)
            return False
            # return BridgeLidar.PATH in self.data

        def update_gps_data(is_alive):
            while is_alive():
                self.data[BridgeGPS.PATH] = json.dumps(
                    {"gps": self.gps.get_cur_gps(), "angle": self.gps.get_cur_angle()}
                )
                time.sleep(1 / config["update_frequency"])

        self._service = Service(update_gps_data, "Actual GPS to Bridge")
        self._service.start_service()
        return True

    def disconnect(self):
        if self.ON_ROVER_SIDE:
            self._service.stop_service()
            self.gps.stop_service()

    def start_service(self):
        pass

    def stop_service(self):
        pass

    def get_cur_angle(self) -> int:
        return json.loads(self.data[BridgeGPS.PATH])["angle"]

    def get_cur_gps(self) -> Tuple[int, int]:
        return json.loads(self.data[BridgeGPS.PATH])["gps"]


if __name__ == "__main__":
    if sys.argv[1] == "r":
        rover_side.service.start_service()
        gps = BridgeGPS()
        gps.connect()
        time.sleep(3)  # give time for threads to start
        while True:
            try:
                print(f"GPS: {gps.get_cur_gps()}, Heading: {gps.get_cur_angle()}")
                time.sleep(1)
            except KeyboardInterrupt:
                print("Keyboard Interrupted")
                break
        gps.disconnect()
        rover_side.service.stop_service()
    elif sys.argv[1] == "c":
        # client_side.service.start_service()
        if not client_side.open_bridge():
            exit(0)
        gps = BridgeGPS()
        try:
            while 1:
                print(f"GPS: {gps.get_cur_gps()}, Heading: {gps.get_cur_angle()}")
                time.sleep(1)
        except KeyboardInterrupt:
            gps.disconnect()
        # client_side.service.stop_service()
        client_side.close_bridge()
