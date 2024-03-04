import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection, PolyCollection
import matplotlib.animation as anim
import json
import sys
import time

config = {
    "blit": False,
    "view_radius_meter": 4,
}
def run_visualizer(get_pathfinder, on_hover_mouse=lambda p:None,):
        fig = plt.figure(0)
        # fig.subplots
        ax = plt.subplot(111, projection='polar')
        tree_lines = LineCollection([], color='g',)
        tree_lines.set_alpha(0.5)
        ax.add_collection(tree_lines)
        path_line = LineCollection([], color='g', linewidths=3)
        ax.add_collection(path_line)
        obstacle_groups = LineCollection([], color='k')
        ax.add_collection(obstacle_groups)
        body = get_pathfinder().worldview.get_rover_body()
        rover_body = PolyCollection([body], closed=True)
        ax.add_collection(rover_body)
        rover_projection = PolyCollection([body], closed=True)
        rover_projection.set_alpha(0.2)
        # ax.add_collection(rover_projection)

        rmax=config['view_radius_meter']
        rscaler = 1.3
        rlagger = 0.99
        ax.set_rmax(rmax)
        def update_plot(_):
            # print()
            modded = []
            # Visualize exploration tree
            tree_links = get_pathfinder().get_tree_links()
            tree_lines.set_segments(tree_links)
            modded.append(tree_lines)
            #---------------------------
            # Visualize path
            path = get_pathfinder().get_path()
            if path:
                path_line.set_segments([path])
                modded.append(path_line)
            #---------------------
            obstacles = get_pathfinder().worldview.get_obstacles()
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

def show_visual(get_pathfinder):
    def on_hover_point(point_polar):
        get_pathfinder().set_goal(point_polar) if not point_polar is None else None
    fig, update_func = run_visualizer(get_pathfinder, on_hover_point)
    anime = anim.FuncAnimation(fig, update_func, 1, interval=50, blit=config['blit'])
    plt.show()
    return anime

if __name__=='__main__':
    import pathfinder
    import worldview
    # pathfinder = StraightShot(worldview)
    # import pathfinder
    import matplotlib
    from unified_utils import time_tracking_service
    time_tracking_service.start_service()
    pathfinder.get_pathfinder().start_pathfinder_service()

    show_visual(pathfinder.get_pathfinder)
    pathfinder.get_pathfinder().stop_pathfinder_service()
    time_tracking_service.stop_service()