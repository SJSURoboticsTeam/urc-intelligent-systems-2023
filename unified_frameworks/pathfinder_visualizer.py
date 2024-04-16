import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection, PolyCollection
import matplotlib.animation as anim
import json
import sys
import time
from unified_utils import polar_sum
import numpy as np
from math import pi

config = {"blit": False, "view_radius_meter": 40, "step_delay": 5}


def run_visualizer(
    get_pathfinder,
    on_hover_mouse=lambda p: None,
):
    fig = plt.figure(0)
    # fig.subplots
    ax = plt.subplot(111, projection="polar")
    goal_points = ax.scatter([0], [1], marker="*", color="g")
    tree_lines = LineCollection(
        [],
        color="g",
    )
    tree_lines.set_alpha(0.5)
    ax.add_collection(tree_lines)
    path_line = LineCollection([], color="g", linewidths=3)
    ax.add_collection(path_line)
    obstacle_groups = LineCollection([], color="k")
    ax.add_collection(obstacle_groups)
    body = get_pathfinder().worldview.get_rover_body()
    rover_body = PolyCollection([body], closed=True)
    ax.add_collection(rover_body)
    rover_projection = PolyCollection([body], closed=True)
    rover_projection.set_alpha(0.2)
    ax.add_collection(rover_projection)

    rmax = config["view_radius_meter"]
    rscaler = 1.3
    rlagger = 0.99
    ax.set_rmax(rmax)
    pos = 1
    delay = 0

    def update_plot(_):
        # print()
        modded = []
        # Goal points
        goal = get_pathfinder().get_goal()
        goal_points.set_offsets([goal])
        modded.append(goal_points)
        # Visualize exploration tree
        tree_links = get_pathfinder().get_tree_links()
        tree_lines.set_segments(tree_links)
        modded.append(tree_lines)
        # ---------------------------
        # Visualize path
        path = get_pathfinder().get_path()
        if path:
            path_line.set_segments([path])
            modded.append(path_line)
        # ---------------------
        obstacles = get_pathfinder().worldview.get_obstacles()
        # Visualizing Obstacles
        obstacle_groups.set_segments(obstacles)
        modded.append(obstacle_groups)
        # ---------------------
        # Animate the rover intention
        nonlocal pos, delay
        delay += 1
        if delay >= config["step_delay"]:
            delay = 0
            pos += 1
        if pos >= len(path):
            pos = 1
        if len(path) > 1:
            # print(len(path),pos)
            a, b = path[pos - 1 : pos + 1]
            rotate_angle = polar_sum(a, (b[0], -b[1]))[0] + pi / 2
            rotated_body = [np.array([rotate_angle, 0]) + p for p in body]
            shifted_body = [polar_sum(rov_p, b) for rov_p in rotated_body]
        else:
            shifted_body = body
        rover_projection.set_verts([shifted_body], closed=True)
        modded.append(rover_projection)

        # Scale for obstacles
        points = sum(obstacles, []) if obstacles is not None else []
        points.extend(path)
        points.extend(sum(tree_links, []))
        points.append(goal)
        nonlocal rmax
        if points:
            rmax = rlagger * rmax + rscaler * (1 - rlagger) * max(
                [d for a, d in points]
            )
            ax.set_rmax(rmax) if not config["blit"] else None

        return modded

    def on_mouse_move(event):
        point_polar = (event.xdata, event.ydata)
        on_hover_mouse(None if any([i is None for i in point_polar]) else point_polar)

    fig.canvas.mpl_connect("motion_notify_event", on_mouse_move)
    return fig, update_plot


def show_visual(get_pathfinder):
    def on_hover_point(point_polar):
        # return
        get_pathfinder().set_goal(point_polar) if not point_polar is None else None

    fig, update_func = run_visualizer(get_pathfinder, on_hover_point)
    anime = anim.FuncAnimation(fig, update_func, 1, interval=50, blit=config["blit"])
    plt.show()
    return anime


if __name__ == "__main__":
    import re
    import os

    root = next(re.finditer(".*unified_frameworks", __file__)).group()
    sys.path.append(root) if root not in sys.path else None
    sys.path.append(os.path.realpath(__file__ + os.sep + ".." + os.sep + ".."))
    import unified_frameworks.pathfinders.pathfinder as _pathfinder
    import matplotlib
    from unified_utils import time_tracking_service

    time_tracking_service.start_service()
    pathfinder = _pathfinder.Pathfinder()
    pathfinder.start_pathfinder_service()

    show_visual(lambda: pathfinder)
    pathfinder.stop_pathfinder_service()
    time_tracking_service.stop_service()
