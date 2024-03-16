import numpy as np
import time
import traceback

try:
    from sensor_array.LidarClass import Lidar
except ModuleNotFoundError:
    import sys
    import re

    sys.path.append((next(re.finditer(".*unified_frameworks", __file__)).group()))
    from sensor_array.LidarClass import Lidar
from rplidar import RPLidar, RPLidarException
import serial.tools.list_ports
from serial.serialutil import PortNotOpenError
from threading import Thread


def getDevicePort():
    ports = serial.tools.list_ports.comports()

    if len(ports) > 0:
        for port, desc, hwid in sorted(ports):
            if hwid != "n/a":
                return port
    return None


class ActualLidar(Lidar):
    def __init__(self, port=getDevicePort()) -> None:
        self.serial_port = port
        self.measures = []

    def connect_to_RPLidar(
        self, max_attempts=3, wait_seconds=1, verbose_attempts=True
    ) -> bool:
        for _ in range(max_attempts):
            print("hi there")
            try:
                print(f"trying to connect on port {self.serial_port}")
                self.lidar = RPLidar(self.serial_port)
                self.lidar_iter = self.lidar.iter_scans()
                next(self.lidar_iter)
                return True
            except RPLidarException as e:
                self.lidar.disconnect()
                continue
            except PortNotOpenError:
                print(f"Actual Lidar Not connected: port {self.serial_port}")
            except:
                print("=================================================")
                print("Lidar Service Failed before lidar could start")
                print(self.serial_port)
                print(traceback.format_exc())
            time.sleep(wait_seconds)
        print("failed")
        return False

    def connect(self, max_attempts=3, wait_seconds=1, verbose_attempts=False) -> bool:
        if not self.connect_to_RPLidar(max_attempts, wait_seconds, verbose_attempts):
            return False
        self.stay_connected = True

        def look():
            while self.stay_connected:
                self.measures = next(self.lidar_iter)

        self.thread = Thread(target=look)
        self.thread.start()
        return True

    def disconnect(self):
        self.stay_connected = False
        self.thread.join()
        lidar = self.lidar
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()

    def get_measures(self):
        return self.measures


if __name__ == "__main__":
    import time

    lidar = ActualLidar()
    lidar.test_Lidar()
