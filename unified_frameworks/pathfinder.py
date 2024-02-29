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
import importlib
importlib.reload(worldview)
from math import pi, cos, sin, atan2, sqrt
from unified_utils import Service
import time
import numpy as np
from shapely import LineString, intersects
import heapq
import json

config = {
    "step_meters": 0.2,
    "initial_radians": pi/2,
    "neighbor_sector": np.array([-1,1])*(60/360)*(2*pi),
    "neighbors": 5,
    "update_frequency": 30, #Hz How frequently to update the shared path and exploration tree
    "explore_frequency": 1000, #Hz How frequently to expand on the exploration tree
    "decimal_precision":5,
    "idx_sectors": 11,
    "shuffle_neighbors": True,
    "verbose_service_events": True,
    "time_analysis": True,
    "heurstic_weight": 3
}

exe_times = {}
def track_time(func):
    def inner(*args, **kwargs):
        begin = time.time()
        res = func(*args, **kwargs)
        exe_times.setdefault(func.__name__, []).append(time.time()-begin)
        return res
    return inner if config['time_analysis'] else func



_goal = (pi/2, 8) # 8 meters at 90 degrees
_path = [] # Path to the point closest to the goal
_tree = [] # Tree of explored area
_prev = None

@track_time
def polar_dis(p1, p2):
    """Distance between polar coordinates p1 and p2"""
    if p1 is None or p2 is None: return float('inf')
    return sqrt(abs(p1[1]**2 + p2[1]**2 - 2*p1[1]*p2[1]*cos(p1[0]-p2[0])))
@track_time
def polar_to_cart(p) -> np.ndarray:
    return np.round(p[1]*np.array([cos(p[0]), sin(p[0])]), config['decimal_precision'])
@track_time
def cart_to_polar(coord):
    x, y = coord
    return np.round((atan2(y, x), np.linalg.norm(coord)),config['decimal_precision'])
@track_time
def heuristic_cost(pos_polar):
    return 0 if _goal is None else polar_dis(pos_polar, _goal)
@track_time
def step_cost(cur, prev):
    if prev is None or cur is None:
            return 0
    return polar_dis(prev, cur)

_backlinks = {None:None}
_arivalcosts = { None: 0 }
_cur = (0,0)
_q = [(heuristic_cost(_cur), _cur, None)]
@track_time
def reset():
    global _backlinks, _arivalcosts, _cur, _q
    _backlinks = {}
    _arivalcosts = { None: 0 }
    _cur = (0,0)
    _q = [(heuristic_cost(_cur), _cur, None)]
@track_time
def get_step_angle(a_polar, b_polar):
    if a_polar is None: return config['initial_radians']
    return cart_to_polar(polar_to_cart(b_polar)-polar_to_cart(a_polar))[0]
@track_time
def get_neighbors(polar_pos):
    cart_pos = polar_to_cart(polar_pos)
    sector_lims = config["neighbor_sector"] + get_step_angle(_backlinks[polar_pos], polar_pos)
    sector_unit = abs(sum(sector_lims*(-1,1)))/config['neighbors']
    angles = [round((sector_lims[0]+i*sector_unit)%(2*pi),2) for i in range(config['neighbors']+1)]
    if angles[0]==angles[-1]: angles.pop()
    polar_vectors = [(a, config['step_meters']) for a in angles]
    cart_neighbors = [polar_to_cart(p) + cart_pos for p in polar_vectors]
    cart_neighbors = np.round(cart_neighbors,config['decimal_precision'])
    polar_neighbors = [cart_to_polar(p) for p in cart_neighbors]
    polar_neighbors = np.round(polar_neighbors,config['decimal_precision'])
    np.random.shuffle(polar_neighbors) if config['shuffle_neighbors'] else None
    return polar_neighbors
@track_time
def get_collision_potential(polar_pos, get_near_points):
    obstacle_points = get_near_points(polar_pos)
    if not obstacle_points: 
        return 0
    min_dis = min((polar_dis(polar_pos, p) for p in obstacle_points))
    pot = 0.1/min_dis**2 if min_dis!=0 else float('inf')
    return pot
@track_time
def check_collision(polar_step, obstacles: list[LineString]):
    if any([i is None for i in polar_step]): return False
    step_string = LineString([polar_to_cart(i) for i in polar_step])
    return any([intersects(step_string, obs) for obs in obstacles])

@track_time
def make_tree():
    global _tree
    _tree = [[k, _backlinks[k]] for k in _backlinks if _backlinks[k] is not None]
@track_time
def make_path():
    if not _backlinks.keys(): return
    near_point = min(_backlinks, key=lambda k: polar_dis(k, _goal))
    path = [near_point]
    while path[-1] is not None:
        path.append(_backlinks[path[-1]])
    path.pop()
    global _path
    _path = list(reversed(path))
    
@track_time
def exploration_step(obstacles:list[LineString], points):
    global _cur
    if not _q: return
    _, _cur, prev = heapq.heappop(_q)
    # if _cur in _backlinks: return
    if any([ (polar_dis(_cur, k) < 0.01) and (polar_dis(prev, _backlinks[k])) for k in _backlinks]): return
    if check_collision((prev, _cur), obstacles):
        return # Kinda sorta handled by avoiding high collision potential
    _backlinks[_cur] = prev
    _arivalcosts[_cur] = _arivalcosts[prev] + step_cost(prev, _cur)
    for n in get_neighbors(_cur):
        pot = get_collision_potential(n, points)
        cost = _arivalcosts[_cur]+step_cost(_cur, n)+heuristic_cost(n)*10
        heapq.heappush(_q,((cost+pot), tuple(n), _cur))


def run_pathfinder(is_pathfinder_running):
    if config['verbose_service_events']:
        print("Starting Pathfinder Service")

    # worldview.get_obstacles = lambda: []
    worldview.start_worldview_service()
    while is_pathfinder_running():
        reset()
        ts = time.time()
        obstacles = worldview.get_obstacles()
        obstacles = [LineString([polar_to_cart(p) for p in obs]) for obs in obstacles if len(obs)>1] if obstacles is not None else []
        points = worldview.get_points()
        sectors = [i*2*pi/config['idx_sectors'] for i in range(config['idx_sectors'])]
        idx = {s:[] for s in sectors}
        for p in ([] if points is None else points):
            # print(p)
            idx[min(sectors, key=lambda s: abs(s-(p[0]%(2*pi))))].append(p)
        near_pts = lambda p: idx[min(sectors, key=lambda s: abs(s-(p[0]%(2*pi))))]
        def get_near_points(polar_pos):
            pts = near_pts(polar_pos)
            i = 0
            while not pts and i < config['idx_sectors']//2:
                i+=1
                pts = near_pts(polar_pos+(i*2*pi/10,0)) + near_pts(polar_pos+(-i*2*pi/10,0))
            if not pts:
                pass
            return pts

        while time.time() - ts < 1/config['update_frequency']:
        # for i in range(20):
            exploration_step(obstacles, get_near_points)
            time.sleep(1/config['explore_frequency'])
        if config['time_analysis']:
            n=10
            time_anal = {i: sum(exe_times[i][-n:])/n for i in exe_times}
            freq_anal = {i: 1/time_anal[i] if time_anal[i]!=0 else float("inf") for i in time_anal}
            with(open("frequency analysis", "w")) as f:
                f.write(json.dumps({"time":time_anal, "freq":freq_anal}, indent=4))
        make_tree()
        make_path()
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
    # print(_path)
    return _path
def get_tree_links():
    return _tree
def set_goal(point_polar):
    global _goal
    _goal = point_polar

if __name__=='__main__':
    pass
    # chart_route(None, None)