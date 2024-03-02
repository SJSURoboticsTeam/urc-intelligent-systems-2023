import pathfinder
import importlib
importlib.reload(pathfinder)
from NavigatorClass import Navigator

class StraightShot(Navigator):
    def __init__(self, worldview) -> None:
        super().__init__(worldview)
        self.goal=None
    def get_path(self):
        return super().get_path() if self.goal is None else [(0,0), self.goal]
    def get_tree_links(self):
        return super().get_tree_links()
    def start_pathfinder_service(self):
        return super().start_pathfinder_service()
    def stop_pathfinder_service(self):
        return super().stop_pathfinder_service()
    def set_goal(self, polar_point):
        self.goal = polar_point