from typing import Tuple
from sensor_array.gps_compass.gps_compass_class import _GPSCompass
import sys
import serial.tools.list_ports as port_list
import re
import os

root = next(re.finditer(".*unified_frameworks", __file__)).group()
sys.path.append(root) if root not in sys.path else None
root = os.path.realpath(os.path.join(root, ".."))
sys.path.append(root) if root not in sys.path else None

# import from modules
from proj_modules import GPS
from proj_modules import LSM303
from threading import Thread


class ActualGPSCompass(_GPSCompass):
    def __init__(self, port: str = None) -> None:
        self.gpsState = True  # Keeps track of the reading state of the GPS
        self.cur_gps = None
        port_number = 0
        ports = list(
            filter(lambda port: "USB" not in port.device, port_list.comports())
        )
        if port in ports:
            self.gps = GPS.gpsRead(port, 57600)
        else:
            print("====> No port specified. Using Port:", ports[port_number].device)
            port = ports[port_number].device
            self.gps = GPS.gpsRead(port, 57600)
            while self.gps.get_position() == [
                "error"
            ] * 2 or self.gps.get_position() == ["None", "None"]:
                print(f"Port {port} not working, going to next port...")
                port_number += 1
                port = ports[port_number].device
                try:
                    self.gps = GPS.gpsRead(port, 57600)
                    break
                except:
                    continue
        self.cur_gps = self.gps.get_position()
        if self.cur_gps is None:
            raise Exception("Unable to get proper GPS lock.")

        self.compass = LSM303.Compass()

        self.gpsThreadCall = Thread(target=self.read)

    def get_cur_angle(self) -> float:
        """
        Returns the latest known heading, relative to North
        """
        return self.compass.get_heading()

    def read(self) -> None:
        """
        On thread for reading gps coordinates
        This way we can continously retrieve the latest data
        """
        while self.gpsState:
            temp = self.gps.get_position()
            if temp is not None:
                self.cur_gps = temp

    def get_cur_gps(self) -> Tuple[int, int]:
        """
        Returns latest GPS coordinates, in the format (longitude, latitude)
        """
        return self.cur_gps

    def start_service(self) -> None:
        """
        Should be called to start the service.
        """
        self.gpsThreadCall.start()

    def stop_service(self) -> None:
        """
        Should be called when we disconnect
        """
        self.gpsState = False
        self.gpsThreadCall.join()


if __name__ == "__main__":
    import time

    gps = ActualGPSCompass()
    gps.start_service()
    try:
        while True:
            print(gps.get_cur_gps(), gps.get_cur_angle())
            time.sleep(1)
    except KeyboardInterrupt:
        gps.stop_service()
