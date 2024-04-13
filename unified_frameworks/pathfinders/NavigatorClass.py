from abc import ABC, abstractmethod
import numpy as np
from typing import Tuple
import math


class Navigator(ABC):
    def __init__(self, worldview) -> None:
        self.worldview = worldview
        self.goal: Tuple[float, float] = (math.pi / 2, 1)  # theta, r

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

    def set_goal(self, polar_point):
        self.goal = polar_point

    def get_goal(self):
        return self.goal

    def distance_to_target(self):
        return self.goal[1]
