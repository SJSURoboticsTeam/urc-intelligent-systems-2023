import sys
import re
from typing import Tuple

root = next(re.finditer(".*unified_frameworks", __file__)).group()
sys.path.append(root) if root not in sys.path else None
from sensor_array.gps_compass.gps_compass_class import GPSCompass
import asyncio
import websockets
from threading import Thread
import json
import time

wifi_address = {
    "robotics_wifi": "ws://192.168.1.130:8777",
    "tp_link": "ws://192.168.0.100:8777",
}


class WirelessGPSCompass(GPSCompass):
    def __init__(self, uri=wifi_address["tp_link"]):
        self.uri = uri
        self.gps = None
        self.angle = None

        self.connect()

    def connect(self, max_attempts=3, verbose_attempts=False) -> bool:
        self.stay_connected = True
        self.connected = False
        self.verbose_attempts = verbose_attempts
        self.thread = Thread(target=self.start_connection)
        self.thread.start()
        for i in range(max_attempts):
            if self.connected:
                return True
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
                        data = await asyncio.wait_for(websocket.recv(), timeout=5)
                        data = json.loads(data)

                        # store results
                        self.gps = data["gps"]
                        self.angle = data["angle"]

                    # in case of not connected
                    self.connected = False
            except asyncio.TimeoutError:
                self.connected = False
                print("GPS update timed out")
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

    def disconnect(self):
        self.stay_connected = False
        for _ in range(10):
            if self.connected:
                print("still connected waiting to disconnect")
                time.sleep(1)

    def get_cur_angle(self) -> int:
        return self.angle

    def get_cur_gps(self) -> Tuple[int, int]:
        return self.gps


if __name__ == "__main__":
    gps = WirelessGPSCompass()
    try:
        while 1:
            print(gps.gps, gps.angle)
            time.sleep(1)
    except KeyboardInterrupt:
        gps.disconnect()
