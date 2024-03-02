"""
This file is meant to be a fake lidar script that should be a 
drop in replacement to the actual RPLidar
I think making this will help in testing an optimizing the lidar scripts
"""
import numpy as np
import time
from sensor_array.LidarClass import Lidar

class FakeLidar(Lidar):
    def __init__(self, port_name, empty_scans) -> None:
        self.n = 30
        self.scan = np.random.rand(self.n, 3)*1_000 +1_000
        self.empty_scans = empty_scans
        pass
    def iter_scans(self1):
        class scan_iterable:
            def __iter__(self):
                return self
            def __next__(self):
                time.sleep(1/4)
                if self.empty_scans:
                    return []
                # print("Faking it")
                # print(self1.scan)
                self1.scan += (0,1,0)
                return self1.scan + (np.random.rand(self1.n, 3)-0.5)*(0,0,400)
                # return self1.scan
        
        return scan_iterable()
    def stop(self):
        pass
    def stop_motor(self):
        pass
    def disconnect(self):
        pass

if __name__=='__main__':
    import time
    lidar = FakeLidar("ws://localhost:8765")
    lidar_iter = lidar.iter_scans()
    start_time = time.time()
    while time.time() - start_time < 10:
        print()
        print(next(lidar_iter))
        time.sleep(0.4)
    lidar.disconnect()
    print("disconnected")