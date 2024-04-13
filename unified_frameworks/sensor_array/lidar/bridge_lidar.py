import sys
import re
root = (next(re.finditer(".*unified_frameworks", __file__)).group())
sys.path.append(root) if root not in sys.path else None
from unified_frameworks.sensor_array.lidar.actual_lidar import ActualLidar
from unified_frameworks.sensor_array.lidar.LidarClass import _Lidar
from bridge import rover_side, client_side
from bridge.exceptions import NoOpenBridgeException
from unified_utils import Service
import json
import time

config = {
    "update_frequency": 20, #Hz
}
class BridgeLidar(_Lidar):
    PATH = '/lidar'
    def __init__(self) -> None:
        if rover_side.bridge_is_up():
            assert not client_side.bridge_is_up(), "Both Rover and Client side bridge is up! Whats going on here??"
            self.lidar = ActualLidar()
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
                if BridgeLidar.PATH in self.data: return True
                time.sleep(wait_seconds)
            return False
            # return BridgeLidar.PATH in self.data
        if not self.lidar.connect(max_attempts, wait_seconds, verbose_attempts):
            return False
        def update_lidar_data(is_alive):
            while is_alive():
                self.data[BridgeLidar.PATH] = json.dumps(self.lidar.get_measures())
                time.sleep(1/config['update_frequency'])
        self._service = Service(update_lidar_data, "Actual Lidar to Bridge Data thread")
        self._service.start_service()
        return True
    def disconnect(self):
        if self.ON_ROVER_SIDE:
            self._service.stop_service()
            self.lidar.disconnect()
    def get_measures(self):
        return json.loads(self.data[BridgeLidar.PATH])
        
        
if __name__=='__main__':
    if sys.argv[1]=='r':
        rover_side.service.start_service()
        lidar = BridgeLidar()
        lidar.connect()
        while True:
            try:
                print(len(lidar.get_measures()))
                time.sleep(1)
            except KeyboardInterrupt:
                print("Keyboard Interrupted")
                break
        lidar.disconnect()
        rover_side.service.stop_service()
    elif sys.argv[1]=='c':
        # client_side.service.start_service()
        if not client_side.open_bridge(): exit(0)
        lidar = BridgeLidar()
        lidar.test_Lidar()
        # client_side.service.stop_service()
        client_side.close_bridge()
