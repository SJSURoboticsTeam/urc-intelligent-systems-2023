import sys
import re
root = (next(re.finditer(".*unified_frameworks", __file__)).group())
sys.path.append(root) if root not in sys.path else None
from sensor_array.LidarClass import Lidar
import asyncio
import websockets
from threading import Thread
import json
import time

wifi_address = {
    "robotics_wifi": "ws://192.168.1.130:8765",
    "tp_link": "ws://192.168.0.104:8765"
}

class WirelessLidar(Lidar):
    def __init__(self, uri=wifi_address["tp_link"]):
        self.uri = uri
        self.data = []
    def connect(self, max_attempts=3, wait_seconds=1, verbose_attempts=False) -> bool:
        self.stay_connected = True
        self.connected = False
        self.verbose_attempts=verbose_attempts
        self.thread = Thread(target=self.start_connection)
        self.thread.start()
        for i in range(max_attempts):
            if self.connected: return True
            time.sleep(1)
        self.disconnect()
        return False
    async def receive_data(self):
        while self.stay_connected:
            try:
                print(f"Trying to connect to websocket at uri `{self.uri}`")
                async with websockets.connect(self.uri) as websocket:
                    self.connected = True
                    while self.stay_connected:
                        data = await asyncio.wait_for(websocket.recv(), timeout=1)
                        # print(data)
                        self.data = json.loads(data)
                    self.connected = False
            except asyncio.TimeoutError:
                self.connected = False
                print("Lidar update timed out")
                time.sleep(1)
                print("Trying again")
            except websockets.exceptions.ConnectionClosedOK:
                self.connected = False
                print("Connection closed")
                time.sleep(1)
                print("Trying again")
    def start_connection(self):
        asyncio.set_event_loop(asyncio.new_event_loop())
        asyncio.get_event_loop().run_until_complete(self.receive_data())
    def get_measures(self):
        return self.data
    def disconnect(self):
        self.stay_connected = False
        for _ in range(10):
            if self.connected:
                print("still connected waiting to disconnect")
                time.sleep(1)
                
    # def iter_scans(self):
    #     return iter(self)
    # def __iter__(self):
    #     return self
    # def __next__(self):
    #     time.sleep(1/4)
    #     return self.data
    # def stop(self):
    #     return super().stop()
    # def stop_motor(self):
    #     return super().stop_motor()
    # def disconnect(self):
    #     self.connected = False
    #     self.thread.join()

if __name__=='__main__':
    WirelessLidar().test_Lidar(3)