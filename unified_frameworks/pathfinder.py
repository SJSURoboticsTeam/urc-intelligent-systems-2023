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
    "step_meters": 0.5,
    "neighbors": 6,
    "initial_radians": pi/2,
    "update_frequency": 5, #Hz How frequently to update the shared path and exploration tree
    "explore_frequency": 600, #Hz How frequently to expand on the exploration tree
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
def polar_addition(p1, p2):
    _p1 = p1[1]*np.array(cos(p1[0]), sin(p1[0]))
    _p2 = p2[1]*np.array(cos(p2[0]), sin(p2[0]))
    res = _p1+_p2
    return (atan2(res[1], res[0]), np.linalg.norm(res))
def heuristic_cost(pos_polar):
    # return 0
    return polar_dis(pos_polar, _goal)
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
    # neighbors = cart_vectors + cart_pos
    # neighbors = [n for n in neighbors if np.linalg.norm(n) < 10] # Only points upto 5m for center
    # neighbors = [n for n in neighbors if not any(
    #     [intersects(so, LineString([pos, n])) for so in shapely_obstacles]
    # )]
    np.random.shuffle(polar_neighbors) if config['shuffle_neighbors'] else None
    return polar_neighbors
def has_obstacle_collision(step_polar, obstacles_polar):
    """Checks if the step (pos A -> pos B) tuple will intersect any obstacle in the obstacle list.
    Inputs are in polar coordinates"""
    if any([i is None for i in step_polar]): return False
    step = LineString([polar_to_cart(p) for p in step_polar])
    obstacles = [
        LineString([polar_to_cart(p) for p in obs])
        for obs in obstacles_polar
    ]
    return any([intersects(step, obs)] for obs in obstacles)
def get_collision_potential(polar_pos, obstacles: list[LineString]):
    # return 0
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

def chart_route(goal, obstacles):
    """Internally using cartesian coordinates, externally everything is polar"""
    a,d = goal
    goal = d*np.array((cos(a), sin(a)))
    cur = (0,0)
    angles = [i * (2*pi/config["neighbors"]) + config["initial_radians"] for i in range(config["neighbors"])]
    vectors = np.array([config["step_meters"]*np.array([cos(a), sin(a)]) for a in angles])
    vectors = np.round(vectors, 2)
    shapely_obstacles = []
    for o in obstacles if obstacles is not None else []:
        o = [d*np.array([cos(a), sin(a)]) for a,d in o]
        if len(o) <= 1: continue
        shapely_obstacles.append(LineString(o))

    def get_neighbors(pos):
        neighbors = np.round((vectors + pos),2)
        neighbors = [n for n in neighbors if np.linalg.norm(n) < 10] # Only points upto 5m for center
        neighbors = [n for n in neighbors if not any(
            [intersects(so, LineString([pos, n])) for so in shapely_obstacles]
        )]
        np.random.shuffle(neighbors)
        return neighbors
    
    def heuristic_cost(pos):
        return np.linalg.norm(goal-pos)*5
    def step_cost(pos, nxt):
        if pos is None:
            return 0
        return np.linalg.norm(np.array(nxt)-pos)
    backlinks = { }
    arival_cost = { None:0 }
    q = [(heuristic_cost(cur),cur, None)]
    iter_lim = 50
    while q and iter_lim > 0:
        iter_lim -= 1
        _, cur, prev = heapq.heappop(q)
        if cur in backlinks:
            continue
        backlinks[cur] = prev
        arival_cost[cur] = arival_cost[prev] + step_cost(prev, cur)
        for n in get_neighbors(cur):
            heapq.heappush(q, (heuristic_cost(n)+arival_cost[cur]+step_cost(cur, n), tuple(n), cur))
    global _tree
    def cart2polar(coord):
        x, y = coord
        return (atan2(y, x), np.linalg.norm(coord))
    _tree = [[cart2polar(k), cart2polar(backlinks[k])] for k in backlinks if backlinks[k] is not None]
    near_point = min(backlinks.keys(), key=lambda i: np.linalg.norm(goal-i))
    path = [near_point]
    while path[-1] is not None:
        path.append(backlinks[path[-1]])
    path.pop()
    global _path
    _path = [cart2polar(p) for p in path]

    pass

def run_pathfinder(is_pathfinder_running):
    if config['verbose_service_events']:
        print("Starting Pathfinder Service")
    
    worldview.start_worldview_service()
    while is_pathfinder_running():
        reset()
        ts = time.time()
        obstacles = worldview.get_obstacles()
        obstacles = [LineString([polar_to_cart(p) for p in obs]) for obs in obstacles if len(obs)>1] if obstacles is not None else []
        while time.time() - ts < 1/config['update_frequency']:
        # for i in range(20):
            exploration_step(obstacles)
            time.sleep(1/config['explore_frequency'])
        make_tree()

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
    chart_route(None, None)