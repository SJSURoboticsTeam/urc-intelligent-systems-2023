from typing import Tuple
from gps_compass.gps_compass_class import GPSCompass
import sys
import serial.tools.list_ports as port_list

root = __file__[: __file__.index("\\unified_frameworks")]
sys.path.append(root + "\\modules")

# import from modules
import GPS
import LSM303


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

    def get_cur_angle(self) -> float:
        return self.compass.get_heading()

    def get_cur_gps(self) -> Tuple[int, int]:
        temp = self.gps.get_position()
        if temp is not None:
            self.cur_gps = temp
        return self.cur_gps


if __name__ == "__main__":
    import time
    gps = ActualGPSCompass()
    try:
        while 1:
            print(gps.gps, gps.angle)
            time.sleep(1)
    except KeyboardInterrupt:
        gps.disconnect()
