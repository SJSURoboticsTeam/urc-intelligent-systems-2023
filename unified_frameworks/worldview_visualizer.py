import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import matplotlib.animation as anim
import json
import sys
# print(sys.path)
import worldview
import time

if __name__=='__main__':
    # worldview.config['']
    worldview.start_worldview_service()
    # time.sleep(20)
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
            # Visualizing Point Cloud
            # point_clouds = worldview.get_point_clouds()
            # measures = sum(point_clouds, []) if point_clouds is not None else []
            # if measures:
            #     obstacle_points.set_offsets(measures)
            #     modded.append(obstacle_points)                                          #
            #---------------------
            # Visualizing Obstacles
            # obstacle_groups.set_segments(obstacles)
            obstacle_groups.set_segments(worldview.get_obstacles())
            modded.append(obstacle_groups)
            #---------------------
            return modded

        anime = anim.FuncAnimation(fig, update_plot, 1, interval=50, blit=True)

        plt.show()
    except:
        pass
    worldview.stop_worldview_service()

