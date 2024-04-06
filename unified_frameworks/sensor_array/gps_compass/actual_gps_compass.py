from typing import Tuple
from gps_compass.gps_compass_class import GPSCompass
import sys
import serial.tools.list_ports as port_list

root = __file__[: __file__.index("\\unified_frameworks")]
sys.path.append(root + "\\modules")

# import from modules
import GPS
import LSM303
from threading import Thread

class ActualGPSCompass(GPSCompass):
    def __init__(self) -> None:
        self.cur_gps = None
        port_number = 0
        ports = list(
            filter(lambda port: "USB" not in port.device, port_list.comports())
        )
        print("====> Designated Port not found. Using Port:", ports[port_number].device)
        port = ports[port_number].device
        self.gps = GPS.gpsRead(port, 57600)
        while (
            self.gps.get_position() == ["error"] * 2 or self.gps.get_position() is None
        ):
            print("Port not found, going to next port...")
            port_number += 1
            port = ports[port_number].device
            try:
                self.gps = GPS.gpsRead(port, 57600)
                break
            except:
                continue
        self.cur_gps = self.gps.get_position()

        self.compass = LSM303.Compass()

        self.gpsThreadCall = Thread(target=self.read)
        self.gpsThreadCall.start()


    def isGPSStateValid(self) -> bool:
        """
        Returns our GPS's current state.
        """
        return self.gpsState

    def get_cur_angle(self) -> float:
        return self.compass.get_heading()

    def read(self) -> None:
        """
        On thread for reading gps coordinates
        This way we can continously retrieve the latest data
        """
        temp = self.gps.get_position()
        if temp is not None:
            self.cur_gps = temp

    def get_cur_gps(self) -> Tuple[int, int]:
        """
        Returns latest GPS coordinates
        """
        return self.cur_gps
    
    def join(self):
        """
        Should be called when we disconnect
        """
        self.gpsThreadCall.join()

if __name__ == "__main__":
    import time
    gps = ActualGPSCompass()
    try:
        while True:
            print(gps.gps, gps.angle)
            time.sleep(1)
    except KeyboardInterrupt:
        gps.disconnect()
        gps.join()

