"""
This script/service will keep an upto date path for navigation 
accounting for surrounding environment and goal


Thoughts on further development
- Currently this limits charting by number of iterations, make it so that its 
    always charting a route and only resets the obstacles every once in a while. 
- Update get neighbors to get neighbors with realistic constraints, like no 
    sharp turns
- Update cost function to have higher cost for points near obstacles.
- ^this can be used to create a potential field, and that potential field could be
    used to update an existing path for a new environment instead of invalidating it 
    due to a new environment
"""

import worldview
from math import pi, cos, sin, atan2, sqrt
from unified_utils import Service
import time
import numpy as np
from shapely import LineString, intersects
import heapq

config = {
    "step_meters": 0.3,
    "neighbors": 6,
    "initial_radians": pi/2,
    "update_frequency": 5, #Hz How frequently to update the shared path and exploration tree
    "explore_frequency": float("inf"), #Hz How frequently to expand on the exploration tree
    "decimal_precision":5,
    "shuffle_neighbors": False,
    "verbose_service_events": True,
}

_goal = (pi/2, 8) # 8 meters at 90 degrees
_path = [] # Path to the point closest to the goal
_tree = [] # Tree of explored area
_prev = None

def polar_dis(p1, p2):
    """Distance between polar coordinates p1 and p2"""
    return sqrt(abs(p1[1]**2 + p2[1]**2 - 2*p1[1]*p2[1]*cos(p1[0]-p2[0])))
def polar_to_cart(p) -> np.ndarray:
    return np.round(p[1]*np.array([cos(p[0]), sin(p[0])]), config['decimal_precision'])
def cart_to_polar(coord):
    x, y = coord
    return np.round((atan2(y, x), np.linalg.norm(coord)),config['decimal_precision'])
def heuristic_cost(pos_polar):
    return 0 if _goal is None else polar_dis(pos_polar, _goal)
def step_cost(cur, prev):
    if prev is None or cur is None:
            return 0
    return polar_dis(prev, cur)

_backlinks = {}
_arivalcosts = { None: 0 }
_cur = (0,0)
_q = [(heuristic_cost(_cur), _cur, None)]
def reset():
    global _backlinks, _arivalcosts, _cur, _q
    _backlinks = {}
    _arivalcosts = { None: 0 }
    _cur = (0,0)
    _q = [(heuristic_cost(_cur), _cur, None)]
def get_neighbors(polar_pos):
    cart_pos = polar_to_cart(polar_pos)
    polar_vectors = [(i*(2*pi/config['neighbors'])+config['initial_radians'], config['step_meters']) for i in range(config['neighbors'])]
    cart_neighbors = [polar_to_cart(p) + cart_pos for p in polar_vectors]
    cart_neighbors = np.round(cart_neighbors,config['decimal_precision'])
    polar_neighbors = [cart_to_polar(p) for p in cart_neighbors]
    polar_neighbors = np.round(polar_neighbors,config['decimal_precision'])
    np.random.shuffle(polar_neighbors) if config['shuffle_neighbors'] else None
    return polar_neighbors
def get_collision_potential(polar_pos, obstacles: list[LineString]):
    if not obstacles: return 0
    pos = polar_to_cart(polar_pos)
    col_points = sum([list(obs.coords) for obs in obstacles], [])
    min_dis = min([np.linalg.norm(pos-col_p) for col_p in col_points])
    return (0.5/min_dis)

def make_tree():
    global _tree
    _tree = [[k, _backlinks[k]] for k in _backlinks if _backlinks[k] is not None]
def make_path():
    global _path
    
def exploration_step(obstacles:list[LineString]):
    global _cur
    if not _q: return
    _, _cur, prev = heapq.heappop(_q)
    # if _cur in _backlinks: return
    if any([polar_dis(_cur, k) < 0.01 for k in _backlinks]): return
    collision = True
    if prev is None:
        collision = False
    if collision:
        step = LineString([polar_to_cart(prev),polar_to_cart(_cur)])
        collision = any([intersects(step, obs) for obs in obstacles])
    if collision:
        return
    _backlinks[_cur] = prev
    for n in get_neighbors(_cur):
        heapq.heappush(_q,(heuristic_cost(n)+get_collision_potential(n, obstacles), tuple(n), _cur))


def run_pathfinder(is_pathfinder_running):
    if config['verbose_service_events']:
        print("Starting Pathfinder Service")

    # worldview.get_obstacles = lambda: []
    worldview.start_worldview_service()
    step_times = []
    steps_measured = 5
    while is_pathfinder_running():
        reset()
        ts = time.time()
        obstacles = worldview.get_obstacles()
        obstacles = [LineString([polar_to_cart(p) for p in obs]) for obs in obstacles if len(obs)>1] if obstacles is not None else []
        while time.time() - ts < 1/config['update_frequency']:
        # for i in range(20):
            t = time.time()
            exploration_step(obstacles)
            time.sleep(1/config['explore_frequency'])
            step_times.append(time.time()-t)
        print(f"Average exploration frequency: {1/sum(step_times[-5:])/5:>5}", end="\r")
        make_tree()
    print()

    worldview.stop_worldview_service()

    if config['verbose_service_events']:
        print("Stopping Pathfinder Service")
    return

_service = Service(run_pathfinder, "Pathfinder service")

def start_pathfinder_service():
    if config['verbose_service_events']:
        print("called start_path_finder_service")
    _service.start_service()
def stop_pathfinder_service():
    if config['verbose_service_events']:
        print("called stop_path_finder_service")
    _service.stop_service()

def get_path():
    return _path
def get_tree_links():
    return _tree

if __name__=='__main__':
    pass
    # chart_route(None, None)