from abc import ABC, abstractmethod

class Navigator(ABC):
    def __init__(self, worldview) -> None:
        self.worldview = worldview
    @abstractmethod
    def get_path(self):
        return []
    @abstractmethod
    def get_tree_links(self):
        return []
    @abstractmethod
    def start_pathfinder_service(self):
        pass
    @abstractmethod
    def stop_pathfinder_service(self):
        pass
    @abstractmethod
    def set_goal(self, polar_point):
        pass
    