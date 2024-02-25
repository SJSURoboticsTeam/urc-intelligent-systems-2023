"""
This script/service will keep an upto date path for navigation 
accounting for surrounding environment and goal
"""

import worldview
from math import pi, cos, sin, atan2
from unified_utils import Service
import time
import numpy as np
from shapely import LineString, intersects
import heapq

config = {
    "step_meters": 1,
    "neighbors": 4,
    "initial_radians": pi/4,
    "update_frequency": 20, #Hz
    "verbose_service_events": True,
}

_goal = (pi/2, 8) # 8 meters at 90 degrees
_path = [(0,0)] # Path to the point closest to the goal
_tree = [] # Tree of explored area
_prev = None

# worldview.get_obstacles = lambda: [[(3*pi/4, 5), (1*pi/4, 5)]]
# worldview.get_obstacles = lambda: []

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
        # print()
        # print(o)
        o = [d*np.array([cos(a), sin(a)]) for a,d in o]
        if len(o) <= 1: continue
        # print()
        # print(o)
        shapely_obstacles.append(LineString(o))
    # shapely_obstacles = [LineString(o) for o in obstacles] if obstacles is not None else []

    def get_neighbors(pos):
        neighbors = np.round((vectors + pos),2)
        neighbors = [n for n in neighbors if np.linalg.norm(n) < 10] # Only points upto 5m for center
        neighbors = [n for n in neighbors if not any(
            [intersects(so, LineString([pos, n])) for so in shapely_obstacles]
        )]
        np.random.shuffle(neighbors)
        return neighbors
    
    backlinks = { }
    q = [(cur, None)]
    while q:
        # print()
        # print(q)
        cur, prev = q.pop(0)
        if cur in backlinks:
            continue
        backlinks[cur] = prev
        q.extend([(tuple(n), cur) for n in get_neighbors(cur)])
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

    # print()
    # print(path)
    # print(np.array(_tree))
    
    
    




    
    


        

    pass

def run_pathfinder(is_pathfinder_running):
    if config['verbose_service_events']:
        print("Starting Pathfinder Service")
    
    worldview.start_worldview_service()
    while is_pathfinder_running():
        chart_route(_goal, worldview.get_obstacles())
        # print("Still pathfinding")
        time.sleep(1/config["update_frequency"])

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
    # start_pathfinder_service()
    # time.sleep(10)
    # stop_pathfinder_service()
    chart_route(None, None)