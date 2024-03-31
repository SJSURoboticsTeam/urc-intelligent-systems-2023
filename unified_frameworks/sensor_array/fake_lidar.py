"""
This file is meant to be a fake lidar script that should be a 
drop in replacement to the actual RPLidar
I think making this will help in testing an optimizing the lidar scripts
"""
import numpy as np
import time
import sys
import re
sys.path.append((next(re.finditer(".*unified_frameworks", __file__)).group()))
from sensor_array.LidarClass import Lidar
from unified_utils import Service, polar_sum

config = {
    "verbose":False
}
class FakeLidar(Lidar):
    def __init__(self, points=100, angular_rate=1, translational_rate=1, jitter=0, noise=0, empty_scans=False) -> None:
        self.n = points
        angles = np.linspace(0, 360, self.n)
        distances = [5_000]
        while len(distances) < self.n:
            shift = 0.5 # if np.random.rand() < 1 else 1
            distances.append(distances[-1]+(np.random.randn()*shift*1_000))
        quality = [15]*self.n
        self.scan = np.stack([quality, angles, distances], 1)
        # self.scan = np.random.rand(self.n, 3)*1_000 +2_000
        print(self.scan)
        self.empty_scans = empty_scans
        self.noise = noise
        def update(is_alive):
            while is_alive():
                self.scan[:, 1]+=(0.7+np.random.randn()*jitter)*angular_rate
                drift = (np.pi/2, 10*translational_rate*(np.e**(jitter*np.random.randn())))
                self.scan[:, 1] = np.deg2rad(self.scan[:, 1])
                self.scan[:, [1,2]] = [polar_sum(i, drift) for i in self.scan[:, [1,2]]]
                self.scan[:, 1] = np.rad2deg(self.scan[:, 1])
                time.sleep(1/10)
        self.s = Service(update, "Fake Lidar Generator")
        pass
    def connect(self, max_attempts=3, wait_seconds=1, verbose_attempts=True) -> bool:
        self.s.start_service()
        return True
    def get_measures(self):
        if self.empty_scans:
            return []
        # self.scan += (0,1,0)
        res = self.scan + (np.random.rand(self.n, 3)-0.5)*(0,0,400)*self.noise
        print(res) if config["verbose"] else None
        return res
        # return self.scan
    def disconnect(self):
        self.s.stop_service()
        pass

if __name__=='__main__':
    FakeLidar().test_Lidar()