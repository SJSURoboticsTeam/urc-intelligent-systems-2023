"""
This is a navigator that implementes an A* stearch in a continous space
to find a path from the current position to the destination position
bla bla bla

"""
import sys
import re
root = (next(re.finditer(".*unified_frameworks", __file__)).group())
sys.path.append(root) if root not in sys.path else None
from numpy import ndarray
from NavigatorClass import Navigator
from math import pi
from shapely import Geometry, intersects, LineString, Point
from heapq import heappop, heappush
import importlib
import unified_utils
from unified_utils import polar_dis, polar_to_cart, polar_sum, same_polar_point, three_point_deviation
importlib.reload(unified_utils)
import numpy as np
from unified_utils import Service, track_time
from time import sleep, time
print("Reloaded")
name="module2"

config = {
    "update_frequency": 10,
    "grow_frequency": 10_000,
    "step_size": 0.2,
    "neighbors": 6,
    "neighbor_sector": [-pi/4, pi/4],
    "obstacle_buffer":0.45
}
_goal = (pi/4, 1)
class A_Star_Navigator(Navigator):
    name="class2"
    def __init__(self, worldview) -> None:
        self.name="instance2"
        self.worldview = worldview
        self._path = []
        self._backlinks = { (0,0): None }
        self._arrival_costs = { None: 0 }
        self._goal = (pi/2, 1)
        self.count=0
        print("Created")
    def get_path(self) -> ndarray:
        return self._path
    def get_tree_links(self) -> ndarray:
        return [[self._backlinks[k], k] for k in self._backlinks if self._backlinks[k] is not None]
    def set_goal(self, polar_point):
        self._goal=polar_point
    def get_goal(self):
        return self._goal
    def distance_to_target(self) -> float:
        return self._goal[1]
    def start_pathfinder_service(self, service_name="A* path finding service"):
        def service_func(is_running):
            while is_running():
                self._backlinks = { (0,0): None } 
                self._arrival_costs = { None: 0 }
                q = [(0,(0,0), None)]
                ts = time()
                while time()-ts < 1/config['update_frequency']:
                    cartesian_obstacles = [[polar_to_cart(p) for p in obs] for obs in self.worldview.get_obstacles() if len(obs)>0]
                    self._grow_tree(q, [LineString(obs).buffer(config['obstacle_buffer']) if len(obs)>1 else Point(obs[0]) for obs in cartesian_obstacles])
                    sleep(1/config['grow_frequency'])
                self._update_path()
        self._service = Service(service_func, service_name)
        self.worldview.start_worldview_service()
        self._service.start_service()
        pass
    def stop_pathfinder_service(self):
        self._service.stop_service()
        self.worldview.stop_worldview_service()
        pass
    def _is_colision(self,cart1, cart2, cart_obstacles: list[Geometry]):
        if cart1 is None or cart2 is None: return False
        coll =  any([intersects(LineString((cart1, cart2)), obs) for obs in cart_obstacles])
        return coll
    def _get_neighbors(self, polar_point):
        p,q = config['neighbor_sector']
        a,b = polar_point, self._backlinks[polar_point]
        initial_angle = pi/2 if b is None else polar_sum(a, (b[0],-b[1]))[0]
        neighbors = [(p+i*(q-p)/(config['neighbors']-1)+initial_angle , config['step_size']) for i in range(config['neighbors'])]
        return [polar_sum(polar_point, n) for n in neighbors]
        pass
    def _update_path(self):
        near_point = min(self._backlinks, key=lambda p: polar_dis(p, self._goal))
        path = [near_point]
        while path[-1] is not None:
            path.append(self._backlinks[path[-1]])
        path.pop()
        self._path = path[::-1]
    def _grow_tree(self, polar_q, cart_obstacles: list[Geometry]=[]):
        if not polar_q: return
        sorting_cost, current_node, previous_node = heappop(polar_q)
        assert current_node is not None, "Got None as current node"
        rpt = any([same_polar_point(current_node, existing_node) for existing_node in self._backlinks])
        if len(self._backlinks) > 1 and rpt:
            return
        coll = self._is_colision(polar_to_cart(previous_node), polar_to_cart(current_node), cart_obstacles)
        if coll:
            return
        assert not coll
        self._backlinks[current_node]=previous_node
        self._arrival_costs[current_node]  = self._arrival_costs[previous_node]
        self._arrival_costs[current_node] += 0 if previous_node is None else polar_dis(previous_node, current_node)
        neighbors = self._get_neighbors(current_node)
        for n in neighbors:
            ac = self._arrival_costs[current_node] + polar_dis(current_node, n)
            hc = polar_dis(n, self._goal)
            tc = abs(three_point_deviation(previous_node, current_node, n))
            cost = ac+3*hc+3*(tc/0.5)**2
            # cost = ac+3*hc
            heappush(polar_q, ((cost), tuple(n), current_node)) 
    
if __name__=='__main__':
    A_Star_Navigator(None)
    print("instantiated without trouble")