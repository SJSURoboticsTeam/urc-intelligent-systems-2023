"""
This script/service will keep an upto date path for navigation 
accounting for surrounding environment and goal


Thoughts on further development
- Currently this limits charting by number of iterations, make it so that its always charting a route and only resets the obstacles every once in a while. 
- Update get neighbors to get neighbors with realistic constraints, like no sharp turns
- Update cost function to have higher cost for points near obstacles.
- ^this can be used to create a potential field, and that potential field could be used to update an existing path for a new environment instead of invalidating it due to a new environment

"""

import sys
import re

root = next(re.finditer(".*unified_frameworks", __file__)).group()
sys.path.append(root) if root not in sys.path else None
import worldview
import importlib

importlib.reload(worldview)
import numpy as np
from typing import Literal

from pathfinders import a_star_navigator
from pathfinders import straight_shot
from pathfinders import rapid_random_tree
from pathfinders.NavigatorClass import Navigator

# Ensures you have instance with latest code reloaded from changes made on the fly
importlib.reload(a_star_navigator)


class Pathfinder(Navigator):
    """
    Top Level Pathfinder
    """

    PATHFINDERS = {
        "a_star": a_star_navigator.A_Star_Navigator,
        "rrt": rapid_random_tree.RRT_Navigator,
        "straight_shot": straight_shot.StraightShot,
    }

    def __init__(
        self, pathfinder: Literal["a_star", "rrt", "straight_shot"] = "a_star"
    ) -> None:
        super().__init__(worldview.Worldview())
        assert pathfinder in Pathfinder.PATHFINDERS
        self.pathfinder: Navigator = Pathfinder.PATHFINDERS[pathfinder](self.worldview)

    def get_path(self) -> np.ndarray:
        return self.pathfinder.get_path()

    def get_tree_links(self) -> np.ndarray:
        return self.pathfinder.get_tree_links()

    def start_pathfinder_service(self):
        self.pathfinder.start_pathfinder_service()

    def stop_pathfinder_service(self):
        self.pathfinder.stop_pathfinder_service()

    def set_goal(self, polar_point):
        self.pathfinder.set_goal(polar_point)

    def get_goal(self):
        return self.pathfinder.get_goal()

    def distance_to_target(self):
        return self.pathfinder.distance_to_target()
