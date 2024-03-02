"""This script will implement helpful functions for RRT. 
It will do all its calculations in cartesian coordinates but it will"""
from unified_utils import track_time, polar_to_cart, cart_to_polar, polar_dis
from shapely import LineString, intersects, Geometry
import numpy as np
import time
from math import pi

config = {
    "step_meters":0.1,
    "rover_radius_meters":0.2,
    "growing_frequency": 1000,
    "update_frequency": 4,
    "verbose":True
}
origin = (0,0)
_parents = {
    origin: None
}
_children = {
    origin: []
}
_tree = []
_path = []
_goal = (pi/2, 1)

@track_time
def reset():
    global _parents, _children
    _parents = {
        origin: None
    }
    _children = {
        origin: []
    }
    # print("reset")

@track_time
def add_node(polar_parent, polar_child):
    polar_parent = tuple(np.round(polar_parent, 2))
    polar_child = tuple(np.round(polar_child, 2))
    # print(f"adding node from {polar_parent} to {polar_child}") if config['verbose'] else None
    # print(_parents)
    assert polar_parent in _parents.keys(), f"{polar_parent}: {_parents}"
    _parents[polar_child] = polar_parent
    _children.setdefault(polar_parent, []).append(polar_child)
@track_time
def heuristic(cart_point, cart_goal):
    return np.linalg.norm(cart_point-cart_goal)
@track_time
def rrt_step(obstacles: list[Geometry], xlim:np.ndarray, ylim:np.ndarray):
    goal = np.random.rand(2)*(abs(sum(xlim*(1,-1))), abs(sum(ylim*(1,-1)))) + (xlim[0], ylim[0])
    # print(f"Random goal: {goal} | lims {xlim}:{ylim} | {(abs(sum(xlim*(1,-1))), abs(sum(ylim*(1,-1))))}") if config['verbose'] else None
    # print(_parents)
    # print(f"Targeting : {goal}")
    # print(f"available points: {_parents.keys()}")
    cartesian_parents = [tuple(polar_to_cart(p)) for p in _parents.keys()]
    values = {p: (-sum([intersects(LineString((p, goal)), obs) for obs in obstacles]), heuristic(p, goal)) for p in cartesian_parents}
    for p in sorted(cartesian_parents, key=lambda p: values[p]):
        # if np.min(np.linalg.norm(cartesian_cloud-p, axis=1)) > config['rover_radius_meters']:
        #     dis = np.linalg.norm(goal-p)
        #     v = (goal-p)/dis
        # print(f"d.coords: {list(d.coords)[0]}") if config['verbose'] else None
        dis = np.linalg.norm(np.array(goal)-p)
        v = ((np.array(goal)-p)/dis)*config['step_meters']
        d = p+v
        # print(f'Considering cart point: {p}')
        if not any([intersects(LineString((p, d)), obs) for obs in obstacles]):
            # print(f"adding {p} -> {d}")
            a = cart_to_polar(p)
            # print(f"a={a}")
            b = cart_to_polar(d)
            # print(f"b={b}")
            add_node(a, b)
            return
    pass
def grow_tree(is_growing:callable, get_polar_obstacles: list[list[list]]):
    while is_growing():
        # print(_parents)

        reset()
        polar_obstacles = get_polar_obstacles()
        cart_obstacles = [[polar_to_cart(p) for p in obs] for obs in polar_obstacles]
        points = np.concatenate(polar_obstacles) if polar_obstacles else [(0,0)]
        buffer = 10
        low_lim = np.min(points, 0) - buffer
        max_lim = np.max(points, 0) + buffer
        obstacles = [LineString(obs).buffer(config["rover_radius_meters"]) for obs in cart_obstacles if len(obs)>1]
        ts = time.time()
        while time.time()-ts < 1/config['update_frequency']:
            rrt_step(obstacles, *np.array([low_lim, max_lim]).T)
            time.sleep(1/config["growing_frequency"])
        global _tree, _path
        _tree = [[k, _parents[k]] for k in _parents if _parents[k] is not None]
        path = [min(_parents, key=lambda p: polar_dis(p, _goal))]
        while path[-1] is not None:
            path.append(_parents[path[-1]])
        path.pop()
        _path = path[::-1]
    pass

from NavigatorClass import Navigator
from unified_utils import Service
class RRT_Navigator(Navigator):
    def __init__(self, worldview) -> None:
        self.worldview = worldview
        self.goal = None
        self._service = Service(grow_tree, "RRT Service", worldview.get_obstacles)
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
    def set_goal(self, polar_point):
        self.goal = polar_point


