from abc import ABC, abstractmethod
class Lidar(ABC):
    @abstractmethod
    def iter_scans(self):
        """
        Get an Iterable that iteratively returns a list of scanned points
        Each Point is a 3 tuple of (quality, angle_degrees, distance_millimeters)
        """
        pass
    @abstractmethod
    def stop(self):
        pass
    @abstractmethod
    def stop_motor(self):
        pass
    @abstractmethod
    def disconnect(self):
        pass