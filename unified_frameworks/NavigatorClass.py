from abc import ABC, abstractmethod
import numpy as np
from worldview import Worldview


class Navigator(ABC):
    def __init__(self, worldview: Worldview) -> None:
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
