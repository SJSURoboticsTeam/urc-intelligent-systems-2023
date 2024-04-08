"""This module has utilities related to making easy visualizations"""

import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.axes import Axes


class Visualizer:
    """
    A visualizer to visualize a bunch of points and lines
    """
    def __init__(self, ax:Axes=None) -> None:
        if ax is None:
            fig, ax = plt.subplots(subplot_kw={'projection':'polar'})
            ax:Axes=ax
        self.points = ax.scatter([],[],s=1)
        ax.set_rmax(50)
        self.ax = ax
    
    def set_points(self, points: list[list[int]]):
        """
        Set points on the visualizer
        
        :param points: list of 2d points
        
        """
        self.points.set_offsets(points)
        rate=0.9
        self.ax.set_rmax(rate*(1.3*max([d for a,d in points]))+(1-rate)*self.ax.get_rmax())
        plt.pause(0.1)


if __name__=='__main__':
    import sys, os
    root = os.path.join(__file__,'..','..','..','unified_frameworks')
    root = os.path.realpath(root)
    sys.path.append(root)
    print()
    print(root)
    from sensor_array.fake_lidar import FakeLidar
    import time
    from math import radians
    lidar = FakeLidar(100)
    lidar.connect()
    try:
        v = Visualizer()
        ts = time.time()
        while plt.get_fignums():
            v.set_points([(radians(a),d/1000) for q,a,d in lidar.get_measures()])
    except Exception as e:
        print(e)
    lidar.disconnect()

