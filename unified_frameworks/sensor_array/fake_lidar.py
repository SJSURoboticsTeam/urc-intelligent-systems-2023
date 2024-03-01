"""
This file is meant to be a fake lidar script that should be a 
drop in replacement to the actual RPLidar
I think making this will help in testing an optimizing the lidar scripts
"""
import numpy as np
import time

class FakeLidar:
    def __init__(self, port_name) -> None:
        self.n = 30
        self.scan = np.random.rand(self.n, 3)*1_000 +1_000
        pass
    def iter_scans(self1):
        class scan_iterable:
            def __iter__(self):
                return self
            def __next__(self):
                time.sleep(1/4)
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