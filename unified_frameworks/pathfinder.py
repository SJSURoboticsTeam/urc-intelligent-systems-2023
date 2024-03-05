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
from unified_utils import Service, track_time
import time
import numpy as np
from shapely import LineString, intersects
import heapq
import json

config = {
    "step_meters": 0.2,
    "initial_radians": pi/3,
    "neighbor_sector": np.array([-1,1])*(90/360)*(2*pi),
    "neighbors": 6,
    "update_frequency": 15, #Hz How frequently to update the shared path and exploration tree
    "explore_frequency": 10000, #Hz How frequently to expand on the exploration tree
    "decimal_precision":5,
    "idx_sectors": 11,
    "shuffle_neighbors": False,
    "verbose_service_events": True,
    "heurstic_weight": 3
}

import a_star_navigator # Import any other navigator you like
importlib.reload(a_star_navigator) # Ensures you have instance with latest code reloaded from changes made on the fly
pf = a_star_navigator.A_Star_Navigator(worldview)
# pf = Pf(worldview)
def get_pathfinder():
    return pf

if __name__=='__main__':
    pass