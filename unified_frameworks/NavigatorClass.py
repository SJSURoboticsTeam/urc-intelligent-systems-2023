from abc import ABC, abstractmethod
import numpy as np

class Navigator(ABC):
    def __init__(self, worldview) -> None:
        self.worldview = worldview
    @abstractmethod
    def get_path(self) -> np.ndarray:
        return []
    @abstractmethod
    def get_tree_links(self) -> np.ndarray:
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
    @abstractmethod
    def get_goal(self):
        pass