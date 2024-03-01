import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import matplotlib.animation as anim
import json
import sys
import time

config = {
    "blit": True
}
def run_visualizer(pathfinder, on_hover_mouse=lambda p:None,):
        fig = plt.figure(0)
        # fig.subplots
        ax = plt.subplot(111, projection='polar')
        tree_lines = LineCollection([], color='g',)
        tree_lines.set_alpha(0.5)
        ax.add_collection(tree_lines)
        path_line = LineCollection([], color='g', linewidths=1)
        ax.add_collection(path_line)
        obstacle_groups = LineCollection([], color='k')
        ax.add_collection(obstacle_groups)

        rmax=4
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

        def on_mouse_move(event):
            point_polar = (event.xdata, event.ydata)
            on_hover_mouse(None if any([i is None for i in point_polar]) else point_polar)
        fig.canvas.mpl_connect("motion_notify_event", on_mouse_move)
        return fig, update_plot

def show_visual(pathfinder):
    def on_hover_point(point_polar):
        pathfinder.set_goal(point_polar) if not point_polar is None else None
    fig, update_func = run_visualizer(pathfinder, on_hover_point)
    anime = anim.FuncAnimation(fig, update_func, 1, interval=50, blit=True)
    plt.show()
    return anime

if __name__=='__main__':
    import pathfinder
    import matplotlib
    pathfinder.start_pathfinder_service()
    def on_hover_point(point_polar):
        pathfinder.set_goal(point_polar) if not point_polar is None else None
    fig, update_func = run_visualizer(pathfinder, on_hover_point)
    # def on_mouse_move(event):
    #     print(event.xdata, event.ydata)
    anime = anim.FuncAnimation(fig, update_func, 1, interval=50, blit=True)
    plt.show()
    pathfinder.stop_pathfinder_service()