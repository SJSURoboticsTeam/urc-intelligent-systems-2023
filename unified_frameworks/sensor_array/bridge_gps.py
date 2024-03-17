from gps_compass.actual_gps_compass import ActualGPSCompass
import time
import sys
import re
root = (next(re.finditer(".*unified_frameworks", __file__)).group())
sys.path.append(root) if root not in sys.path else None
from bridge import rover_side, client_side
from bridge.exceptions import NoOpenBridgeException
from unified_utils import Service

import json

config = {
    "update_frequency": 2, #Hz
}

class BridgeGPS:
    PATH = '/gps'
    def __init__(self) -> None:
        super().__init__()
        if rover_side.bridge_is_up():
            assert not client_side.bridge_is_up(), "Both Rover and Client side bridge is up! Whats going on here??"
            self.gps = ActualGPSCompass()
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
                if BridgeGPS.PATH in self.data: return True
                time.sleep(wait_seconds)
            return False
            # return BridgeLidar.PATH in self.data
        def update_gps_data(is_alive):
            while is_alive():
                print(f"Data updated {self.data}")
                self.data[BridgeGPS.PATH] = json.dumps(
                    {"gps": self.gps.get_cur_gps(), "angle": self.gps.get_cur_angle()}
                )
                time.sleep(1/config['update_frequency'])
        self._service = Service(update_gps_data, "Actual GPS to Bridge")
        self._service.start_service()
        return True
    def disconnect(self):
        if self.ON_ROVER_SIDE:
            self._service.stop_service()
    def get_measures(self):
        return json.loads(self.data[BridgeGPS.PATH]) if BridgeGPS in self.data else f"Not yet Set {self.data}"


if __name__=='__main__':
    if sys.argv[1]=='r':
        rover_side.service.start_service()
        gps = BridgeGPS()
        gps.connect()
        while True:
            try:
                # time.sleep(2)
                print(gps.get_measures())
                time.sleep(1)
            except KeyboardInterrupt:
                print("Keyboard Interrupted")
                break
        gps.disconnect()
        rover_side.service.stop_service()
    elif sys.argv[1]=='c':
        # client_side.service.start_service()
        if not client_side.open_bridge(): exit(0)
        gps = BridgeGPS()
        try:
            while 1:
                print(gps.get_measures())
                time.sleep(1)
        except KeyboardInterrupt:
            gps.disconnect()
        # client_side.service.stop_service()
        client_side.close_bridge()

