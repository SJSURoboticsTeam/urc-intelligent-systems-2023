import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import matplotlib.animation as anim
import json
import sys
# print(sys.path)
# import pathfinder
import time

config = {
    "blit": True
}
# worldview.config['']
def run_visualizer(pathfinder):
# if True:
    # pathfinder.start_pathfinder_service()
    # time.sleep(20)
    try:
        fig = plt.figure()
        ax = plt.subplot(111, projection='polar')
        tree_lines = LineCollection([], color='g',)
        tree_lines.set_alpha(0.5)
        ax.add_collection(tree_lines)
        path_line = LineCollection([], color='g', linewidths=3)
        ax.add_collection(path_line)
        obstacle_groups = LineCollection([], color='k')
        ax.add_collection(obstacle_groups)

        rmax=10
        rscaler = 1.1
        rlagger = 0.99
        ax.set_rmax(rmax)
        def update_plot(_):
            # print()
            modded = []
            # Visualize exploration tree
            tree_links = pathfinder.get_tree_links()
            tree_lines.set_segments(tree_links)
            modded.append(tree_lines)
            #---------------------------
            # Visualize path
            path = pathfinder.get_path()
            if path:
                path_line.set_segments([path])
                modded.append(path_line)
            #---------------------
            obstacles = pathfinder.worldview.get_obstacles()
            # Visualizing Obstacles
            obstacle_groups.set_segments(obstacles)
            modded.append(obstacle_groups)
            #---------------------
            # Scale for obstacles
            points = sum(obstacles, []) if obstacles is not None else []
            points.extend(path)
            points.extend(sum(tree_links, []))
            nonlocal rmax
            if points:
                rmax = rlagger*rmax + rscaler*(1-rlagger)*max([d for a,d in points])
                ax.set_rmax(rmax) if not config['blit'] else None

            return modded

        # anime = anim.FuncAnimation(fig, update_plot, 1, interval=50, blit=config['blit'])
        # plt.show()
        # return anim
        return fig, update_plot

    except:
        pass
    # pathfinder.stop_pathfinder_service()

# anime = run_visualizer()
# plt.show()

if __name__=='__main__':
    import pathfinder
    pathfinder.start_pathfinder_service()
    fig, update_func = run_visualizer(pathfinder)
    anime = anim.FuncAnimation(fig, update_func, 1, interval=50, blit=True)
    plt.show()
    pathfinder.stop_pathfinder_service()