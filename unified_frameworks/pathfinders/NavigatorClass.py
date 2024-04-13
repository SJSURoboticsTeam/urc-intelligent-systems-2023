from abc import ABC, abstractmethod
import numpy as np
from typing import Tuple
import math
from worldview import Worldview


class Navigator(ABC):
    def __init__(self, worldview: Worldview) -> None:
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

    def set_gps_goal(self, target_latitude: float, target_longitude: float) -> None:
        """
        Set's the navigator's goal to a given GPS coordinate. Must be called frequently in
        order to continually head in the right direction.
        """
        self.set_goal(
            self.worldview.geographic_coordinates_to_relative_coordinates(
                target_latitude, target_longitude
            )
        )
