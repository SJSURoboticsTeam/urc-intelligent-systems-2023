"""This script will implement helpful functions for RRT. 
It will do all its calculations in cartesian coordinates but it will"""

from unified_utils import track_time, polar_to_cart, cart_to_polar, polar_dis
from shapely import LineString, intersects, Geometry
import numpy as np
import time
from math import pi

config = {
    "step_meters": 0.1,
    "rover_radius_meters": 0.2,
    "growing_frequency": 1000,
    "update_frequency": 4,
    "verbose": True,
}
_path = []
_tree = []
node_vals = [(0, 0)]
_backlinks = {0: None}


def exploration_step(obstacles: list[LineString]):
    rpoint = np.random.rand(2) * (2 * pi, 10)
    nearest_node = min(
        range(len(node_vals)), key=lambda i: polar_dis(node_vals[i], rpoint)
    )
    if any(
        [
            intersects(
                LineString(
                    (polar_to_cart(node_vals[nearest_node]), polar_to_cart(rpoint))
                ),
                obs,
            )
            for obs in obstacles()
        ]
    ):
        return
    _backlinks[len(node_vals)] = nearest_node
    node_vals.append(rpoint)
    global _tree
    _tree.append([node_vals[nearest_node], (rpoint)])


def grow_tree(is_growing, obstacles):
    while is_growing():
        exploration_step(obstacles)
        time.sleep(0.5)


from unified_frameworks.pathfinders.NavigatorClass import Navigator
from unified_utils import Service


class RRT_Navigator(Navigator):
    def __init__(self, worldview) -> None:
        self.worldview = worldview
        self.goal = None
        self._service = Service(
            grow_tree,
            "RRT Service",
            lambda: [LineString(obs) for obs in worldview.get_obstacles],
        )

    def get_path(self) -> np.ndarray:
        return _path

    def get_tree_links(self) -> np.ndarray:
        # print(f"Tree: {_tree}")
        return _tree

    def start_pathfinder_service(self):
        self.worldview.start_worldview_service()
        self._service.start_service()

    def stop_pathfinder_service(self):
        self._service.stop_service()
        self.worldview.stop_worldview_service()
