"""
This file is meant to be a fake lidar script that should be a 
drop in replacement to the actual RPLidar
I think making this will help in testing an optimizing the lidar scripts
"""
import numpy as np
import time
try:
    from sensor_array.LidarClass import Lidar
except ModuleNotFoundError:
    import sys
    import re
    sys.path.append((next(re.finditer(".*unified_frameworks", __file__)).group()))
    from sensor_array.LidarClass import Lidar

class FakeLidar(Lidar):
    def __init__(self, empty_scans=False) -> None:
        self.n = 30
        self.scan = np.random.rand(self.n, 3)*1_000 +1_000
        self.empty_scans = empty_scans
        pass
    def connect(self, max_attempts=3, wait_seconds=1, verbose_attempts=True) -> bool:
        return True
    def get_measures(self):
        if self.empty_scans:
            return []
        self.scan += (0,1,0)
        return self.scan + (np.random.rand(self.n, 3)-0.5)*(0,0,400)
    def disconnect(self):
        pass

if __name__=='__main__':
    FakeLidar().test_Lidar()