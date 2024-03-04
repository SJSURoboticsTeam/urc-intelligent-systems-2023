from numpy import ndarray
from NavigatorClass import Navigator
from math import pi
from shapely import Geometry, intersects
from heapq import heappop, heappush
import importlib
import unified_utils
from unified_utils import polar_dis, polar_to_cart, polar_sum, same_polar_point
importlib.reload(unified_utils)
import numpy as np
from unified_utils import Service, track_time
from time import sleep, time
print("Reloaded")
name="module2"

config = {
    "update_frequency": 1,
    "grow_frequency": 10_000,
    "step_size": 0.2
}
_goal = (pi/4, 1)
class A_Star_Navigator(Navigator):
    name="class2"
    def __init__(self, worldview) -> None:
        self.name="instance2"
        self.worldview = worldview
        self._path = []
        # self._tree = [
        #     # [(0,0), (0,0)],
        # ]
        self._backlinks = {}
        self._goal = np.array((pi/2, 1))
        print("Created")
    def get_path(self) -> ndarray:
        return self._path
    def get_tree_links(self) -> ndarray:
        # return self._tree
        return [[self._backlinks[k], k] for k in self._backlinks if self._backlinks[k] is not None]
    def set_goal(self, polar_point):
        self._goal=polar_point
    def start_pathfinder_service(self, service_name="A* path finding service"):
        def service_func(is_running):
            while is_running():
                # self._tree.clear()
                self._backlinks.clear()
                q = [(0,(0,0), None)]
                ts = time()
                while time()-ts < 1/config['update_frequency']:
                    self._grow_tree(q)
                    sleep(1/config['grow_frequency'])
        self._service = Service(service_func, service_name)
        self.worldview.start_worldview_service()
        self._service.start_service()
        pass
    def stop_pathfinder_service(self):
        self._service.stop_service()
        self.worldview.stop_worldview_service()
        pass
    def _is_colision(self,cart1, cart2, cart_obstacles: list[Geometry]):
        return False
    def _get_neighbors(self, polar_point):
        neighbors = [(i*2*pi/6, config['step_size']) for i in range(6)]
        return [polar_sum(polar_point, n) for n in neighbors]
        pass
    def _grow_tree(self, polar_q, cart_obstacles: list[Geometry]=[]):
        self._path = ((0,0), self._goal)
        if not polar_q: return
        sorting_cost, current_node, previous_node = heappop(polar_q)
        assert current_node is not None, "Got None as current node"
        rpt = any([same_polar_point(current_node, existing_node) for existing_node in self._backlinks])
        # rpt = any([same_polar_point(current_node, existing_node) for existing_node in [b for a,b in self._tree]])
        if len(self._backlinks) > 1 and rpt:
            return
        if self._is_colision(None, polar_to_cart(current_node), cart_obstacles):
            return
        self._backlinks[current_node]=previous_node
        # self._tree.append([previous_node, current_node]) if previous_node is not None else None
        neighbors = self._get_neighbors(current_node)
        for n in neighbors:
            heappush(polar_q, (len(self._backlinks), tuple(n), current_node))
        
    
if __name__=='__main__':
    A_Star_Navigator(None)
    print("instantiated without trouble")