import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import matplotlib.animation as anim
import json
import sys
import worldview as w
import time
import traceback

if __name__=='__main__':
    worldview = w.Worldview()
    worldview.start_service()
    try:
        fig = plt.figure()
        ax = plt.subplot(111, projection='polar')
        obstacle_points = ax.scatter([],[], s=1)
        obstacle_groups = LineCollection([], color=(0,0,0,0.5), linewidths=5)
        ax.add_collection(obstacle_groups)

        rmax=10
        rmax_=0.01
        ax.set_rmax(rmax)
        def update_plot(_):
            modded = []
            obstacle_groups.set_segments(worldview.get_obstacles())
            modded.append(obstacle_groups)
            return modded

        anime = anim.FuncAnimation(fig, update_plot, 1, interval=50, blit=True)

        plt.show()
    except Exception as e:
        print(e)
        traceback.print_exc()

        pass
    worldview.stop_service()

